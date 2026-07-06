"""
任务有限状态机引擎。

管理任务队列，循环执行各状态，
每次迭代执行全局健康检查，并处理错误/超时。
"""

import asyncio
import time
from typing import Optional

from interface import PX4Interface
from config import FSM_LOOP_HZ
from logger_manager import get_logger
from states.base_state import BaseState
from states.takeoff import TakeoffState
from states.hover import HoverState
from states.transit import TransitState
from states.land_in_place import LandInPlaceState


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
            HoverState(hover_time=5.0),
            TransitState(x=5.0, y=0.0, z=5.0, speed=5.0, timeout_s=30),
            LandInPlaceState(timeout_s=60),
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
            prev_state = self.current_state.name if self.current_state else "start"
            get_logger().log_state_transition(prev_state, state.name)
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
                get_logger().log_message(
                    "warning", f"{state.name} 超时 ({state.timeout_s}秒)，跳过",
                    "timeout")

            self.state_index += 1

        # 任务结束 —— 以飞控实际状态为准，确保安全断开上锁
        await self._ensure_disarmed()

    # ------------------------------------------------------------------
    # 安全收尾
    # ------------------------------------------------------------------

    async def _ensure_disarmed(self):
        """
        确保飞控已安全断开上锁。

        正常流程中，PX4 Land 模式着陆后会自动 disarm；
        此方法检测实际状态，必要时发送 disarm 并等待确认。
        若 disarm 失败，尝试 RTL 作为最后手段。
        """
        if not await self._is_armed():
            print("[信息] 任务完成，飞控已断开上锁")
            get_logger().log_message("info", "任务完成，飞控已断开上锁")
            return

        # 仍在上锁状态 —— 发送 disarm 并等待确认
        print("[信息] 发送 disarm 指令...")
        await self.interface.disarm()

        if await self._wait_for_disarm(timeout=5.0):
            print("[信息] 任务完成，飞控已断开上锁")
            get_logger().log_message("info", "任务完成，飞控已断开上锁")
            return

        # disarm 失败 —— 降级为 RTL
        print("[错误] disarm 失败，飞控未响应 —— 尝试 RTL 作为最后手段")
        get_logger().log_message(
            "error", "disarm 失败，飞控未响应，尝试 RTL", "fail")
        try:
            await self.interface.drone.action.return_to_launch()
        except Exception as e:
            print(f"[致命错误] RTL 也失败了: {e}")
            get_logger().log_message(
                "fatal", f"RTL 失败: {e}", "fail")

    async def _is_armed(self) -> bool:
        """查询飞控当前是否处于上锁状态（单次快照）。"""
        async for armed in self.interface.drone.telemetry.armed():
            return armed

    async def _wait_for_disarm(self, timeout: float = 5.0) -> bool:
        """等待飞控断开上锁，返回 True 表示成功断开。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not await self._is_armed():
                return True
            await asyncio.sleep(0.1)
        return False

    async def _handle_unhealthy(self, reason: str):
        """紧急情况：健康检查失败时触发 RTL 自主返航降落。"""
        print(f"[紧急] 全局健康检查失败: {reason}")
        print("[紧急] 触发 RTL 返航 —— PX4 将自主爬升、返航、降落")
        get_logger().log_message(
            "emergency", f"全局健康检查失败: {reason}", "fail")
        get_logger().log_message(
            "emergency", "触发 RTL 返航 —— PX4 将自主爬升、返航、降落")
        await self.interface.drone.action.return_to_launch()

    async def _handle_state_error(self, state: BaseState, error: Exception):
        """状态执行错误的统一处理。"""
        print(f"[错误] {state.name} 抛出异常: {error}")
        print("[紧急] 触发 RTL 返航")
        get_logger().log_message(
            "error", f"{state.name} 抛出异常: {error}", "fail")
        get_logger().log_message(
            "emergency", "触发 RTL 返航")
        await self.interface.drone.action.return_to_launch()
