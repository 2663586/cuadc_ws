"""
投放后导航状态 —— 投放完成后的安全上升与返航。

destination 参数:
  "return"  — 上升 → 飞回中断点 (shared["position_left"]) → 悬停 → 完成
  "recon"   — 上升 → 直飞侦察区起点 → 悬停 → 完成

4 个子步骤:
  Step 0: 垂直上升至巡航高度 (CRUISE_ALTITUDE_M)
  Step 1: 水平飞向目的地 (中断点 或 侦察区起点)
  Step 2: 悬停稳定 (1 秒)
  Step 3: 返回 done，触发栈弹出链
"""

import math
import time

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult
from .recon import RECON_FORWARD_M, RECON_RIGHT_HALF
from config import CRUISE_ALTITUDE_M, TRANSIT_SPEED_MPS


class PostDropNavState(BaseState):
    """投放后：上升 → 返航(或去侦察区) → 悬停 → 完成。"""

    POST_HOVER_TIME = 1.0
    ASCEND_COMPLETE_MARGIN = 0.5

    def __init__(self, timeout_s: float = 60,
                 destination: str = "return"):
        super().__init__("PostDropNav", timeout_s)
        self._destination = destination  # "return" | "recon"
        self._step = 0
        self._step_start = 0.0
        self._step_init = True
        self._ascent_north = 0.0
        self._ascent_east = 0.0
        self._original_vel_max = None

    async def enter(self, interface):
        await super().enter(interface)
        self._step = 0
        self._step_start = 0.0
        self._step_init = True

        pos = await interface.get_position_ned()
        self._ascent_north = pos.north_m
        self._ascent_east = pos.east_m

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "投放后导航超时"
            return ExecutionResult(done=True)

        if self._step == 0:
            return await self._do_ascend(interface)
        elif self._step == 1:
            return await self._do_return(interface)
        elif self._step == 2:
            return await self._do_hover(interface)
        elif self._step == 3:
            self.is_completed = True
            return ExecutionResult(done=True)
        return ExecutionResult()

    async def exit(self, interface):
        if self._original_vel_max is not None:
            await interface.restore_cruise_speed(self._original_vel_max)
        await super().exit(interface)

    async def _do_ascend(self, interface):
        if self._step_init:
            self._step_init = False
            self._step_start = time.monotonic()
            sp = PositionNedYaw(
                self._ascent_north, self._ascent_east,
                -CRUISE_ALTITUDE_M, interface.FIELD_YAW_DEG,
            )
            interface.update_setpoint(sp)
            print(f"[导航] Step 0: 上升至 {CRUISE_ALTITUDE_M}m")

        alt = await interface.get_altitude()
        if abs(alt - CRUISE_ALTITUDE_M) < self.ASCEND_COMPLETE_MARGIN:
            print(f"[导航] Step 0 完成, 高度:{alt:.1f}m")
            self._step = 1
            self._step_init = True
        return ExecutionResult()

    async def _do_return(self, interface):
        """Step 1: 飞向目的地 (中断点 或 侦察区起点)。"""
        # ---- 确定目标点 ----
        if self._destination == "recon":
            # 侦察区扫描起点 (场地坐标 → NED)
            # field_to_ned(north=forward, east=right, up=height)
            target_ned = interface.field_to_ned(
                RECON_FORWARD_M, -RECON_RIGHT_HALF, CRUISE_ALTITUDE_M)
        else:
            # 中断点
            pos_left = interface.shared.get("position_left")
            if pos_left is None:
                print("[导航] Step 1 跳过 (无中断点)")
                self._step = 2
                self._step_init = True
                return ExecutionResult()
            target_ned = PositionNedYaw(
                pos_left.north_m, pos_left.east_m,
                -CRUISE_ALTITUDE_M, interface.FIELD_YAW_DEG,
            )

        # ---- 首次进入: 发布飞行指令 ----
        if self._step_init:
            self._step_init = False
            self._step_start = time.monotonic()
            self._original_vel_max = await interface.start_position_flight(
                target_ned, TRANSIT_SPEED_MPS)
            dest_name = "侦察区起点" if self._destination == "recon" else "中断点"
            print(f"[导航] Step 1: 飞往{dest_name} "
                  f"N({target_ned.north_m:.1f}) E({target_ned.east_m:.1f})")

        # ---- 检查到达 ----
        pos = await interface.get_position_ned()
        h_dist = math.hypot(
            target_ned.north_m - pos.north_m,
            target_ned.east_m - pos.east_m,
        )
        alt = await interface.get_altitude()

        if h_dist < 0.5 and abs(alt - CRUISE_ALTITUDE_M) < self.ASCEND_COMPLETE_MARGIN:
            print(f"[导航] Step 1 完成 (剩余 {h_dist:.1f}m)")
            self._step = 2
            self._step_init = True
        return ExecutionResult()

    async def _do_hover(self, interface):
        if self._step_init:
            self._step_init = False
            self._step_start = time.monotonic()
            print(f"[导航] Step 2: 悬停 {self.POST_HOVER_TIME}s")

        if time.monotonic() - self._step_start > self.POST_HOVER_TIME:
            print("[导航] Step 2 完成")
            self._step = 3
        return ExecutionResult()
