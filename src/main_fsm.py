"""
任务有限状态机引擎。

管理任务队列，循环执行各状态，
每次迭代执行全局健康检查，并处理错误/超时。
"""

import asyncio
import time
from typing import Optional

from mavsdk.offboard import PositionNedYaw

from interface import PX4Interface
from config import FSM_LOOP_HZ
from states.base_state import BaseState
from states.takeoff import TakeoffState
from states.hover import HoverState
from states.land import PrecisionLandState


class MissionFSM:
    """任务状态机引擎。"""

    def __init__(self, interface: PX4Interface):
        self.interface = interface
        self.mission_queue: list[BaseState] = []
        self.current_state: Optional[BaseState] = None
        self.state_index = 0

        # 注册不健康回调
        interface._on_unhealthy = self._handle_unhealthy

    def build_mission(self):
        """构建任务序列。"""
        self.mission_queue = [
            TakeoffState(target_alt=5.0, timeout_s=30),
            HoverState(hover_time=1.0),
            PrecisionLandState(timeout_s=60),
        ]
        self.state_index = 0

    async def run(self):
        """主状态机循环。"""
        self.build_mission()
        await self.interface.connect_and_setup()
        await self.interface.arm_and_offboard()

        while self.state_index < len(self.mission_queue):
            state = self.mission_queue[self.state_index]
            self.current_state = state

            # ---- 全局健康检查 ----
            if not await self.interface.global_guard_check():
                await self._handle_unhealthy("进入状态前全局守卫失败")
                break

            # ---- 进入状态 ----
            await state.enter(self.interface)

            # ---- 执行循环 ----
            while True:
                if not await self.interface.global_guard_check():
                    await self._handle_unhealthy("状态执行中健康检查失败")
                    return

                try:
                    done, _ = await state.execute(self.interface)
                except Exception as e:
                    state.error = str(e)
                    await self._handle_state_error(state, e)
                    break

                if done:
                    break
                await asyncio.sleep(1.0 / FSM_LOOP_HZ)

            # ---- 退出状态 ----
            await state.exit(self.interface)

            # ---- 超时处理 ----
            if state.is_timed_out() and not state.is_completed:
                print(f"[警告] {state.name} 超时 ({state.timeout_s}秒)，"
                      f"跳过")

            self.state_index += 1

        # 任务完成
        await self.interface.disarm()
        print("[信息] 任务完成")

    async def _handle_unhealthy(self, reason: str):
        """紧急情况：健康检查失败时强制悬停。"""
        print(f"[紧急] 全局健康检查失败: {reason}")
        self.interface.update_setpoint(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        )

    async def _handle_state_error(self, state: BaseState, error: Exception):
        """状态执行错误的统一处理。"""
        print(f"[错误] {state.name} 抛出异常: {error}")
        self.interface.update_setpoint(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        )
