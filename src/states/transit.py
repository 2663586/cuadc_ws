"""巡航状态 —— 以受控速度飞往场地 NED 坐标航点。

控制架构（增量位置斜坡 + PX4 位置控制）：

  不做的事：直接发送速度指令（set_velocity_ned）
  → 速度指令绕过了 PX4 的 Position Controller，丧失了位置反馈和
    高度保持能力。20Hz Python 循环无法与 PX4 内部 200Hz+ 的
    级联控制器竞争。

  做的事：每周期将位置 setpoint 向目标推进一小步
  → PX4 的 Position Controller 将 setpoint 与 EKF 估计位置比较，
    生成速度指令 → Velocity Controller 跟踪 → 飞控自动保持高度、
    限制加速度。Python 侧只负责"告诉 PX4 去哪里"。

  为什么高度稳定：PX4 Position Controller 的 Z 轴直接运行在 EKF
  融合高度上（IMU + 气压计 + GPS），采样率和控制率都是几百 Hz，
  远非 Python 侧 20Hz + gRPC 延迟可比。

参见：
  - https://docs.px4.io/main/en/flight_modes/offboard
  - https://discuss.px4.io/t/a-few-questions-about-velocity-setpoint/29796
"""

import math
import time

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult
from config import ARRIVAL_REL_THRESHOLD, FSM_LOOP_HZ


class TransitState(BaseState):
    """
    以受控速度飞往目标场地 NED 坐标。

    参数 north, east, up 均在"场地 NED"坐标系中：
      - north = 沿场地前方方向（米）
      - east  = 沿场地右方方向（米）
      - up    = 飞行高度（米，向上为正）

    工作方式：
      每周期（1/FSM_LOOP_HZ 秒）将 setpoint 从 ramp 位置
      向目标推进 step = speed/FSM_LOOP_HZ 米。PX4 跟踪此
      移动 setpoint，实际飞行速度 ≈ speed。
    """

    def __init__(self, north: float, east: float, up: float,
                 speed: float = 5.0, timeout_s: float = 60):
        super().__init__("Transit", timeout_s)
        self.target_north, self.target_east, self.target_up = north, east, up
        self.speed = speed

        # 状态（在 enter() 中初始化）
        self._target_ned = None
        self._total_dist = None
        self._ramp = None        # PositionNedYaw, 斜坡当前位置
        self._debug_counter = 0
        # 诊断：追踪上次位置以检测无人机是否在移动
        self._last_drone_north = None
        self._last_drone_east = None
        self._debug_t0 = 0.0

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def enter(self, interface):
        await super().enter(interface)

        self._target_ned = interface.field_to_ned(
            self.target_north, self.target_east, self.target_up)

        # 斜坡从无人机当前位置开始
        pos = await interface.get_position_ned()
        self._ramp = PositionNedYaw(
            pos.north_m, pos.east_m,
            self._target_ned.down_m,    # Z 直接 snap 到目标高度
            self._target_ned.yaw_deg,
        )

        self._total_dist = math.hypot(
            self._target_ned.north_m - pos.north_m,
            self._target_ned.east_m - pos.east_m,
        )

        step = self.speed / FSM_LOOP_HZ
        print(f"[巡航] -> 场地 NED({self.target_north:.1f}, "
              f"{self.target_east:.1f}, {self.target_up:.1f}) "
              f"@ {self.speed:.1f} 米/秒, 总距 {self._total_dist:.1f} 米, "
              f"步进 {step:.2f} 米/周期")

        self._debug_t0 = time.monotonic()

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "巡航超时"
            return ExecutionResult(done=True)

        # ---- 无人机当前位置（仅用于到达判据，不参与 ramp 计算） ----
        pos = await interface.get_position_ned()
        drone_dist = math.hypot(
            self._target_ned.north_m - pos.north_m,
            self._target_ned.east_m - pos.east_m,
        )
        alt = await interface._read_altitude_direct()

        # ---- 到达判据：无人机已足够接近目标 ----
        # 最低 0.3m 阈值防止零距离/极短距离场景下判据永远不触发
        arrive_threshold = max(self._total_dist * ARRIVAL_REL_THRESHOLD, 0.3)
        if drone_dist < arrive_threshold and abs(alt - self.target_up) < 0.5:
            # Ramp snap 到目标，位置保持
            interface.update_setpoint(self._target_ned)
            # 直接发送，确保 setpoint 立即到达 PX4（不依赖心跳转发）
            await interface.drone.offboard.set_position_ned(self._target_ned)
            self.is_completed = True
            return ExecutionResult(done=True)

        # ---- 增量斜坡：推进 ramp 位置 ----
        step = self.speed / FSM_LOOP_HZ

        # 重新计算 ramp 到目标的方向和距离
        dn_ramp = self._target_ned.north_m - self._ramp.north_m
        de_ramp = self._target_ned.east_m - self._ramp.east_m
        ramp_dist = math.hypot(dn_ramp, de_ramp)

        if ramp_dist < step:
            # 最后一步：直接 snap 到目标
            self._ramp = PositionNedYaw(
                self._target_ned.north_m, self._target_ned.east_m,
                self._target_ned.down_m, self._target_ned.yaw_deg,
            )
        else:
            # 向目标方向推进 step 距离
            self._ramp = PositionNedYaw(
                self._ramp.north_m + (dn_ramp / ramp_dist) * step,
                self._ramp.east_m  + (de_ramp / ramp_dist) * step,
                self._target_ned.down_m,
                self._target_ned.yaw_deg,
            )

        # 发送位置 setpoint —— 直接发送，不依赖心跳转发
        interface.update_setpoint(self._ramp)
        await interface.drone.offboard.set_position_ned(self._ramp)

        # ---- 调试输出（约 1 Hz） ----
        self._debug_counter += 1
        if self._debug_counter % FSM_LOOP_HZ == 0:
            pct = drone_dist / self._total_dist * 100 if self._total_dist > 0 else 0
            elapsed = time.monotonic() - self._debug_t0
            avg_hz = self._debug_counter / elapsed if elapsed > 0 else 0

            # 无人机实际移动速率（基于 NED 位置变化）
            drone_speed_s = 0.0
            if self._last_drone_north is not None:
                dn = pos.north_m - self._last_drone_north
                de = pos.east_m - self._last_drone_east
                drone_speed_s = math.hypot(dn, de) * FSM_LOOP_HZ  # 估算瞬时速率

            print(f"[巡航] 剩余 {drone_dist:.1f}m ({pct:.0f}%)  "
                  f"ramp→target {ramp_dist:.1f}m  alt={alt:.1f}m  "
                  f"无人机移速≈{drone_speed_s:.2f}m/s  "
                  f"FSM估算={avg_hz:.1f}Hz")
            self._last_drone_north = pos.north_m
            self._last_drone_east = pos.east_m

        return ExecutionResult()
