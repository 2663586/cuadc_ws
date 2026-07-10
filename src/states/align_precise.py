"""
精细对准状态 —— 速度视觉伺服 + 角速度稳定检查。

从 interface.shared 读取目标位置（粗定位），
下降至约 2.5m 后通过 YOLO 实时检测进行精细对准。

伺服策略:
  距离 > PID_ENGAGE_DISTANCE_M  → P 控制 (NED 偏移 → 速度)
  距离 ≤ PID_ENGAGE_DISTANCE_M  → PID 控制 (像素差 → 速度)

两个子阶段:
  descend  — 位置控制下降至 DROP_ALIGN_ALTITUDE_M
  servo    — 视觉伺服 (P 逼近 → PID 精调)

对准完成后将结果写入 shared，返回 done=True，
由上游状态统一调度下一步（投放）。
"""

import math
import time

from mavsdk.offboard import PositionNedYaw, VelocityNedYaw

from .base_state import BaseState, ExecutionResult
from config import (
    DROP_ALIGN_ALTITUDE_M,
    DROP_ZONE_DISTANCE_M,
)
from pid_velocity import VelocityPID
from vision.camera import capture_frame_async
from vision.circle_detector import DEFAULT_CAMERA_MATRIX
from vision.pipeline import VisionPipeline


# 相机内参 — 与 search.py 中 pixel_to_ned_offset 使用的值保持一致
_FX = float(DEFAULT_CAMERA_MATRIX[0, 0])
_FY = float(DEFAULT_CAMERA_MATRIX[1, 1])
_CX = float(DEFAULT_CAMERA_MATRIX[0, 2])
_CY = float(DEFAULT_CAMERA_MATRIX[1, 2])
_BUCKET_HEIGHT_M = 0.30


def _pixel_to_ned(cx_px: float, cy_px: float, alt_rel_m: float):
    """下视相机像素坐标 → 场地 NED 偏移 (north_m, east_m)。

    图像上方=场地北，图像右方=场地东。
    """
    z_c = alt_rel_m - _BUCKET_HEIGHT_M
    if z_c <= 0:
        return (0.0, 0.0)
    dx = cx_px - _CX
    dy = cy_px - _CY
    east_m = dx * z_c / _FX
    north_m = -dy * z_c / _FY
    return (north_m, east_m)


class AlignPreciseState(BaseState):
    """速度视觉伺服精细对准 + 角速度稳定判断。

    从 shared["drop_targets"] 读取粗定位，下降至 2.5m
    后用实时 YOLO 检测进行速度伺服:
      - 距离 > PID_ENGAGE_DISTANCE_M: P 控制逼近
      - 距离 ≤ PID_ENGAGE_DISTANCE_M: PID 像素级精调
    对准完成后返回 done=True，由上游状态调度投放。

    skip_descend: True 时跳过下降阶段，直接进入速度伺服
                  (RollTaskState 已将无人机降至 DROP_ALIGN_ALTITUDE_M)。
    """

    # ---- 速度伺服参数 ----
    KP_VEL = 2.0                    # P 控制增益 (距离→速度, 逼近阶段)
    PID_ENGAGE_DISTANCE_M = 0.05    # 进入 PID 精调的距离阈值 (m)
    MAX_VEL = 1.0                   # 水平速度上限 (m/s)
    CYCLE_INTERVAL = 0.2            # 视觉检测节流 (s)，约 5 Hz
    ANGULAR_VEL_THRESHOLD = 0.5     # 机体角速度稳定阈值 (rad/s)
    DISTANCE_THRESHOLD = 0.02       # 对准完成距离阈值 (m)
    ALIGN_TIMEOUT = 30.0            # 对准阶段超时 (s)，超时也视为完成

    def __init__(self, bottle_index: int, timeout_s: float = 60,
                 skip_descend: bool = False):
        super().__init__("AlignPrecise", timeout_s)
        self.bottle_index = bottle_index
        self._skip_descend = skip_descend

        # 子阶段
        self.phase = "servo" if skip_descend else "descend"
        self._target = None           # 缓存的 CylinderTarget
        self._align_start = 0.0
        self._last_detect_time = 0.0
        self._pid_engaged = False     # True = 已切入 PID 精调模式

        # PID 控制器 (NED 偏移 m → 速度 m/s, 两轴独立)
        self._pid_north = VelocityPID(
            kp=0.6, ki=0.15, kd=0.2, max_out=self.MAX_VEL)
        self._pid_east = VelocityPID(
            kp=0.6, ki=0.15, kd=0.2, max_out=self.MAX_VEL)

        # 视觉流水线（实时 YOLO 检测）
        self._pipeline = VisionPipeline(
            model_path=None,
            yolo_conf=0.5,
            circle_conf_threshold=0.3,
        )

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def enter(self, interface):
        await super().enter(interface)

        # ---- 从 shared 读取粗定位目标 ----
        targets = interface.shared.get("drop_targets")
        idx = self.bottle_index - 1
        if targets is None or targets[idx] is None:
            self.error = f"共享缓存中没有瓶子 {self.bottle_index} 的检测结果"
            print(f"[精细对准] {self.error}")
            return

        self._target = targets[idx]

        if self._skip_descend:
            # RollTaskState 已将无人机降至 DROP_ALIGN_ALTITUDE_M
            # 直接进入速度伺服
            self.phase = "servo"
            self._align_start = time.monotonic()
            self._last_detect_time = 0.0
            print(f"[精细对准] 瓶子 {self.bottle_index}: "
                  f"跳过下降，直接开始速度伺服")
        else:
            # ---- 阶段1: 下降到精细对准高度 ----
            sp = interface.field_to_ned(
                DROP_ZONE_DISTANCE_M + self._target.ned_offset[0],
                self._target.ned_offset[1],
                DROP_ALIGN_ALTITUDE_M,
            )
            interface.update_setpoint(sp)
            self.phase = "descend"
            print(f"[精细对准] 瓶子 {self.bottle_index}: "
                  f"下降至 {DROP_ALIGN_ALTITUDE_M:.1f}m")

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "精细对准超时"
            interface.clear_velocity()
            return ExecutionResult(done=True)

        if self._target is None:
            return ExecutionResult(done=True)

        if self.phase == "descend":
            return await self._do_descend(interface)
        elif self.phase == "servo":
            return await self._do_servo(interface)
        return ExecutionResult()

    # ------------------------------------------------------------------
    # 阶段 1: 下降
    # ------------------------------------------------------------------

    async def _do_descend(self, interface):
        """等待高度降至 DROP_ALIGN_ALTITUDE_M。"""
        alt = await interface.get_altitude()
        if alt <= DROP_ALIGN_ALTITUDE_M + 0.3:
            self.phase = "servo"
            self._align_start = time.monotonic()
            self._last_detect_time = 0.0
            print(f"[精细对准] 瓶子 {self.bottle_index}: "
                  f"到达 {alt:.1f}m, 开始速度伺服")
        return ExecutionResult()

    # ------------------------------------------------------------------
    # 阶段 2: 速度视觉伺服
    # ------------------------------------------------------------------

    async def _do_servo(self, interface):
        """单节拍速度伺服 (不阻塞 FSM)。"""

        now = time.monotonic()

        # ---- 节流 ----
        dt_since_last = now - self._last_detect_time
        if dt_since_last < self.CYCLE_INTERVAL:
            return ExecutionResult()
        self._last_detect_time = now

        elapsed = now - self._align_start

        # ---- 超时: 停止伺服, 视为完成 ----
        if elapsed > self.ALIGN_TIMEOUT:
            print(f"[精细对准] 超时 ({self.ALIGN_TIMEOUT}s), 强制完成")
            interface.clear_velocity()
            self.is_completed = True
            return ExecutionResult(done=True)

        # ---- 1. 角速度稳定性 ----
        angular_vel = 0.0
        try:
            async for rate in interface.drone.telemetry.attitude_angular_velocity_body():
                angular_vel = math.sqrt(
                    rate.roll_rad_per_s ** 2 +
                    rate.pitch_rad_per_s ** 2 +
                    rate.yaw_rad_per_s ** 2
                )
                break
        except Exception as e:
            print(f"[精细对准] 角速度获取失败: {e}")
            return ExecutionResult()

        # ---- 2. 视觉检测 ----
        try:
            alt = await interface.get_altitude()
            frame = await capture_frame_async()
            results = self._pipeline.process_frame(frame, alt_rel_m=alt)
        except Exception as e:
            print(f"[精细对准] 视觉检测异常: {e}")
            return ExecutionResult()

        # 提取成功检测到的圆柱体 (保留像素坐标供 PID 使用)
        cylinders = []
        for r in results:
            if r["edge_success"]:
                circle = r["circle"]
                ned = _pixel_to_ned(circle.cx_px, circle.cy_px, alt)
                cylinders.append({
                    "ned_offset": ned,
                    "distance": math.hypot(ned[0], ned[1]),
                    "diameter_m": r["diameter_m"],
                    "cx_px": circle.cx_px,          # 圆心像素 x
                    "cy_px": circle.cy_px,          # 圆心像素 y
                })

        if not cylinders:
            interface.update_velocity_setpoint(
                VelocityNedYaw(0.0, 0.0, 0.0, interface.FIELD_YAW_DEG))
            print("[精细对准] 未检测到圆柱体")
            return ExecutionResult()

        # 取最近圆柱体
        best = min(cylinders, key=lambda c: c["distance"])
        dx, dy = best["ned_offset"]
        distance = best["distance"]
        dx_px = best["cx_px"] - _CX    # 圆心→图像中心像素差 (东)
        dy_px = best["cy_px"] - _CY    # 圆心→图像中心像素差 (下=南)

        # ---- 3. 对准判据 (不变) ----
        if (distance < self.DISTANCE_THRESHOLD
                and angular_vel < self.ANGULAR_VEL_THRESHOLD):
            print(f"[精细对准] OK → "
                  f"距离:{distance:.3f}m 角速度:{angular_vel:.3f}rad/s "
                  f"耗时:{elapsed:.1f}s")
            interface.shared[f"bottle_{self.bottle_index}_position"] = best
            interface.clear_velocity()
            self._pid_north.reset()
            self._pid_east.reset()
            self._pid_engaged = False
            self.is_completed = True
            return ExecutionResult(done=True)

        # ---- 4. 速度控制: 近距离 PID / 远距离 P ----
        if distance <= self.PID_ENGAGE_DISTANCE_M:
            # -------- PID 精调 (NED 偏移 m → 速度 m/s) --------
            if not self._pid_engaged:
                self._pid_north.reset()
                self._pid_east.reset()
                self._pid_engaged = True
                print(f"[精细对准] 距离 {distance:.2f}m ≤ "
                      f"{self.PID_ENGAGE_DISTANCE_M}m，切入 PID 精调")

            v_north = self._pid_north.update(dx, now)
            v_east = self._pid_east.update(dy, now)
        else:
            # -------- P 控制逼近 (NED 偏移 → 速度) --------
            if self._pid_engaged:
                self._pid_engaged = False
                print(f"[精细对准] 距离 {distance:.2f}m > "
                      f"{self.PID_ENGAGE_DISTANCE_M}m，切回 P 控制")

            v_north = dx * self.KP_VEL
            v_east = dy * self.KP_VEL
            speed = math.hypot(v_north, v_east)
            if speed > self.MAX_VEL:
                v_north *= self.MAX_VEL / speed
                v_east *= self.MAX_VEL / speed

        interface.update_velocity_setpoint(
            VelocityNedYaw(v_north, v_east, 0.0, interface.FIELD_YAW_DEG))

        mode = "PID" if self._pid_engaged else "P"
        print(f"[精细对准] 调整中[{mode}] "
              f"偏移:({dx:+.3f},{dy:+.3f})m "
              f"像素差:({dx_px:+.0f},{dy_px:+.0f})px "
              f"速度:({v_north:+.2f},{v_east:+.2f})m/s "
              f"距离:{distance:.3f}m")
        return ExecutionResult()
