"""
精细对准与投放状态 —— 视觉伺服 + 投放 + 投放后导航。

在 2.5m 高度通过视觉伺服将无人机精确对准圆柱体中心,
满足条件后投放载荷, 然后上升返航并根据完成情况路由到
侦察或继续矩形搜索。支持两轮投放 (15cm / 20cm 各一次)。
"""

import math
import time

from mavsdk.offboard import PositionNedYaw, VelocityNedYaw

from .base_state import BaseState
from .drop import DropState
from .align import _capture_frame_async
from config import (
    CRUISE_ALTITUDE_M,
    TRANSIT_SPEED_MPS,
)
from vision.yolo_detector import get_detector


class AlignPreciseState(BaseState):
    """精细对准、投放、投放后导航。支持两轮投放。"""

    def __init__(self, bottle_index: int, timeout_s: float = 120):
        super().__init__("AlignPrecise", timeout_s)
        self.bottle_index = bottle_index

    # ---- 可调参数 ----
    # 精细对准 (速度伺服)
    ALIGN_TIMEOUT = 30.0           # 最大耗时, 超时强制投放 (s)
    ANGULAR_VEL_THRESHOLD = 0.5    # 机体角速度低于此值认为已稳定 (rad/s)
    DISTANCE_THRESHOLD = 0.05      # 视觉中心与圆筒中心距离低于此值认为对准 (m)
    CYCLE_INTERVAL = 0.2           # 视觉检测间隔, 节流避免 YOLO 过载 (s)
    KP_VEL = 2.0                   # 速度伺服增益: 偏移(m) × KP = 速度(m/s)
    MAX_VEL = 1.0                  # 水平速度上限 (m/s)
    # 投放后导航
    ASCEND_COMPLETE_MARGIN = 0.5   # 高度到位判据: 当前高度与目标之差 < 此值 (m)
    POST_HOVER_TIME = 1.0          # 到达中断点后悬停等待机身稳定 (s)

    # ================================================================
    # 生命周期: enter → execute (循环) → exit
    # ================================================================

    async def enter(self, interface):
        """
        状态进入点。重置全部内部变量, 使同一状态实例可被两轮投放复用。

        从 interface 读取当前 NED 位置作为视觉伺服起点,
        从 _last_setpoint 抓取当前水平锚点供投放后上升使用。
        """
        await super().enter(interface)

        # -- 阶段切换: False=还在对准, True=已投放, 进入导航 --
        self._drop_done = False

        # -- 对准节拍变量: 由 _align_tick 首次调用时实际初始化 --
        self._align_initialized = False   # 首 tick 门卫, 只跑一次 init
        self._align_start = 0.0           # 对准开始时间戳 (s)
        self._last_check_time = 0.0       # 上次视觉检测时间, 用于节流
        self._detected_cylinder = None    # 最近一次检测到的圆柱体对象

        # -- 导航子步骤: 0=上升 1=返航 2=悬停 3=分支 --
        self._nav_step = 0
        self._nav_step_start = 0.0        # 当前子步骤的开始时间
        self._nav_step0_init = True       # Step 0 首次进入标记 (只发布一次设定值)
        self._nav_step1_init = True       # Step 1 首次进入标记

        # -- 上升锚点: 先用当前 setpoint, 投放时 _do_drop 会更新为投放位置 --
        self._ascent_north = interface._last_setpoint.north_m
        self._ascent_east = interface._last_setpoint.east_m

        print(f"[投放] 第{self.bottle_index}号瓶, 进入对准阶段 "
              f"(速度伺服, KP={self.KP_VEL})")

    # ================================================================
    # 主循环 — 每 FSM 周期调用一次, 不阻塞
    # ================================================================

    async def execute(self, interface):
        """
        两阶段调度:
          A. _drop_done == False → 运行 _align_tick 一个节拍
          B. _drop_done == True  → 运行 _post_drop_navigation 推进一个子步骤

        路由方式: 由于 FSM 的 execute() 返回值中的 next_state 被丢弃,
        本状态改为在 interface.shared["next_phase"] 写入 "recon" 或
        "rectangle", 由 FSM 或 Router 状态读取后分派。
        """
        # ---- 阶段 A: 视觉伺服对准 → 投放 ----
        if not self._drop_done:
            self._drop_done = await self._align_tick(interface)
            return False, None

        # ---- 阶段 B: 投放后导航 ----
        done, _ = await self._post_drop_navigation(interface)
        if done:
            self.is_completed = True
        return done, None

    # ================================================================
    # _align_tick — 单节拍视觉伺服 (不阻塞 FSM 健康检查)
    # ================================================================

    async def _align_tick(self, interface):
        """
        对准循环的一个节拍, 每次 FSM tick 调用后立即返回。

        流程:
          首次 → 初始化计时器和 goal
          节流 → 距上次检测不足 CYCLE_INTERVAL 则跳过
          超时 → 强制投放
          获取角速度 + 视觉检测 → 判断对准
          未对准 → P 控制调整 NED 设定值

        返回 True 表示投放已执行, 可切换至阶段 B。
        """
        now = time.monotonic()

        # -- 首 tick 初始化: 只跑一次 --
        if not self._align_initialized:
            self._align_initialized = True
            self._align_start = now                # 对准计时起点
            self._last_check_time = 0.0            # 置 0 确保首次立即检测
            self._goal = interface.shared.setdefault("goal", [0, 0])
            # ↑ setdefault: 第二轮投放时 goal 已有第一轮的值, 不会被覆盖
            self._detected_cylinder = None

        # -- 节流: 控制视觉检测频率 --
        if now - self._last_check_time < self.CYCLE_INTERVAL:
            return False
        self._last_check_time = now

        elapsed = now - self._align_start

        # -- 超时保护: 先停速再投放 --
        if elapsed > self.ALIGN_TIMEOUT:
            print(f"[对准] 超时 ({self.ALIGN_TIMEOUT}s), 强制投放")
            interface.update_velocity(VelocityNedYaw(0.0, 0.0, 0.0, interface.FIELD_YAW_DEG))
            await self._do_drop(interface)
            return True

        # -- 1. 获取机体三轴角速度, 计算合成幅值 --
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
            print(f"[对准] 角速度获取失败: {e}")
            return False

        # -- 2. 视觉检测: 高度用于像素→米制换算, 圆柱体位置完全由实时画面反算 --
        try:
            alt = await interface.get_altitude()
            detector = get_detector()
            frame = await _capture_frame_async()
            cylinders = detector.detect_cylinders(frame, alt)
        except Exception as e:
            print(f"[对准] 视觉检测异常: {e}")
            return False

        if not cylinders:
            print("[对准] 未检测到圆柱体")
            interface.update_velocity(VelocityNedYaw(0.0, 0.0, 0.0, interface.FIELD_YAW_DEG))
            return False

        # 取距离摄像头中心最近的圆柱体 (最可能为本轮目标)
        self._detected_cylinder = min(
            cylinders, key=lambda c: math.hypot(c.ned_offset[0], c.ned_offset[1]))
        dx, dy = self._detected_cylinder.ned_offset   # 摄像头→圆筒中心的 NED 偏移 (m)
        distance = math.hypot(dx, dy)                  # 水平距离

        # -- 3. 对准判据: 距离足够小 且 机身足够稳定 --
        if distance < self.DISTANCE_THRESHOLD and angular_vel < self.ANGULAR_VEL_THRESHOLD:
            print(f"[对准] OK -> "
                  f"距离:{distance:.3f}m 角速度:{angular_vel:.3f}rad/s "
                  f"耗时:{elapsed:.1f}s")
            # 切回位置模式 (发送零速→心跳自动保持位置)
            interface.update_velocity(VelocityNedYaw(0.0, 0.0, 0.0, interface.FIELD_YAW_DEG))
            await self._do_drop(interface)
            return True

        # -- 4. 未对准: 速度伺服, 偏移越大速度越快, 偏移→0 速度自然→0 --
        v_north = dx * self.KP_VEL
        v_east  = dy * self.KP_VEL

        # 限速: 合成速度不超过 MAX_VEL
        speed = math.hypot(v_north, v_east)
        if speed > self.MAX_VEL:
            v_north *= self.MAX_VEL / speed
            v_east  *= self.MAX_VEL / speed

        interface.update_velocity(
            VelocityNedYaw(v_north, v_east, 0.0, interface.FIELD_YAW_DEG))
        # ↑ 心跳循环以 20Hz 持续发送此速度, 飞控驱动电机执行

        print(f"[对准] 调整中... "
              f"偏移:({dx:+.3f},{dy:+.3f})m "
              f"速度:({v_north:+.2f},{v_east:+.2f})m/s "
              f"距离:{distance:.3f}m 角速度:{angular_vel:.3f}rad/s "
              f"耗时:{elapsed:.1f}s")
        return False

    # ================================================================
    # _do_drop — 投放执行 + goal 更新 + 锚点刷新
    # ================================================================

    async def _do_drop(self, interface):
        """
        投放后三件事:
          1. 调用 DropState 释放舵机
          2. 根据检测到的圆筒直径更新 goal[0] 或 goal[1]
          3. 刷新上升锚点为当前伺服位置 (避免 Step 0 上升前多一段水平回退)
        """
        drop_state = DropState(bottle_index=self.bottle_index)
        await drop_state.execute(interface)
        print(f"[投放] 第{self.bottle_index}号瓶: 投放完成")

        # goal[0]=1 表示 15cm 圆筒已投放, goal[1]=1 表示 20cm 圆筒已投放
        if self._detected_cylinder is not None:
            dia = self._detected_cylinder.diameter
            if abs(dia - 0.15) < 0.02:
                self._goal[0] = 1
                print("[记录] goal[0]=1 (15cm 圆筒完成)")
            elif abs(dia - 0.20) < 0.02:
                self._goal[1] = 1
                print("[记录] goal[1]=1 (20cm 圆筒完成)")
            else:
                print(f"[记录] 未知直径 {dia:.3f}m, 不更新 goal")

        # 锚点跟进: 读取投放时实际位置, 上升时直接从这里起
        pos = await interface.get_position_ned()
        self._ascent_north = pos.north_m
        self._ascent_east = pos.east_m

    # ================================================================
    # _post_drop_navigation — 投放后 4 步子步骤状态机
    # ================================================================

    async def _post_drop_navigation(self, interface):
        """
        每 FSM 周期推进一个子步骤, 直至完成:

          Step 0  垂直上升至 CRUISE_ALTITUDE_M, 保持水平位置不变
          Step 1  飞回矩形航线中断点 position_left
          Step 2  悬停 POST_HOVER_TIME (1s) 等待机身稳定
          Step 3  检查 goal 是否全部完成, 写入 shared["next_phase"] 路由标记
        """
        goal = interface.shared.get("goal", [0, 0])
        pos_left = interface.shared.get("position_left")  # 矩形航线中断时保存的无人机场地坐标

        # ----------------------------------------------------------------
        # Step 0: 垂直上升。首次进入发布设定值, 之后每 tick 检查高度。
        # ----------------------------------------------------------------
        if self._nav_step == 0:
            if self._nav_step0_init:
                self._nav_step0_init = False
                self._nav_step_start = time.monotonic()
                # 保持投放后水平位置, 仅将 down 分量改为巡航高度
                sp = PositionNedYaw(
                    self._ascent_north, self._ascent_east,
                    -CRUISE_ALTITUDE_M, interface.FIELD_YAW_DEG,
                )
                interface.update_setpoint(sp)
                print(f"[导航] Step 0: 上升至 {CRUISE_ALTITUDE_M}m")

            alt = await interface.get_altitude()
            if abs(alt - CRUISE_ALTITUDE_M) < self.ASCEND_COMPLETE_MARGIN:
                print(f"[导航] Step 0 完成, 高度:{alt:.1f}m")
                self._nav_step = 1
                self._nav_step_start = time.monotonic()
            return False, None

        # ----------------------------------------------------------------
        # Step 1: 飞回中断点。首次发布 field_to_ned 设定值, 之后按时间+高度判断到达。
        # ----------------------------------------------------------------
        if self._nav_step == 1:
            if pos_left is None:
                print("[导航] 警告: position_left 未保存, 跳过 Step 1")
                self._nav_step = 2
                self._nav_step_start = time.monotonic()
                return False, None

            if self._nav_step1_init:
                self._nav_step1_init = False
                self._nav_step_start = time.monotonic()
                # pos_left 为场地坐标 (forward, right), 通过 field_to_ned 转换
                sp = interface.field_to_ned(
                    forward=pos_left[0], right=pos_left[1],
                    height=CRUISE_ALTITUDE_M)
                interface.update_setpoint(sp)
                print(f"[导航] Step 1: 飞回中断点 "
                      f"({pos_left[0]:.1f}, {pos_left[1]:.1f})")

            alt = await interface.get_altitude()
            # 从上升锚点 (当前 NED) 到 pos_left (NED) 的实际飞行距离
            pos_left_sp = interface.field_to_ned(
                pos_left[0], pos_left[1], CRUISE_ALTITUDE_M)
            dist = math.hypot(
                pos_left_sp.north_m - self._ascent_north,
                pos_left_sp.east_m - self._ascent_east,
            )
            est_time = dist / TRANSIT_SPEED_MPS if TRANSIT_SPEED_MPS > 0 else 10
            step_elapsed = time.monotonic() - self._nav_step_start

            # 水平距离: 当前位置 → pos_left
            pos = await interface.get_position_ned()
            h_dist = (
                (pos_left_sp.north_m - pos.north_m) ** 2 +
                (pos_left_sp.east_m  - pos.east_m)  ** 2
            ) ** 0.5

            # 到达判据: 水平距离 < 0.5m 且 高度接近目标
            # (时间作为兜底, 防止 odometry 漂移)
            if (h_dist < 0.5
                    and abs(alt - CRUISE_ALTITUDE_M) < self.ASCEND_COMPLETE_MARGIN):
                print(f"[导航] Step 1 完成 (飞行约 {dist:.1f}m)")
                self._nav_step = 2
                self._nav_step_start = time.monotonic()
            elif step_elapsed > est_time * 1.5:
                print(f"[导航] Step 1 超时 (距离 {h_dist:.1f}m), 强制推进")
                self._nav_step = 2
                self._nav_step_start = time.monotonic()
            return False, None

        # ----------------------------------------------------------------
        # Step 2: 悬停稳定。不发布新设定值, 仅等待时间到达。
        # ----------------------------------------------------------------
        if self._nav_step == 2:
            if time.monotonic() - self._nav_step_start > self.POST_HOVER_TIME:
                print(f"[导航] Step 2 完成 (悬停 {self.POST_HOVER_TIME}s)")
                self._nav_step = 3
            return False, None

        # ----------------------------------------------------------------
        # Step 3: 分支路由。根据 goal 决定下一步去向。
        # ----------------------------------------------------------------
        if self._nav_step == 3:
            if goal[0] == 1 and goal[1] == 1:
                print("[导航] goal 全部完成 -> recon")
                interface.shared["next_phase"] = "recon"
            else:
                print(f"[导航] goal={goal}, 未全部完成 -> rectangle")
                interface.shared["next_phase"] = "rectangle"
            return True, None

        return True, None  # fallback
