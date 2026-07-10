"""
投放状态 —— 舵机释放 + 去向判定。

对准已在 AlignPreciseState 中完成。
投放后根据 bottle_index 判定去向：
  - bottle_index==1 → PostDropNavState("return") → 回中断点 → SearchState 继续巡逻
  - bottle_index==2 → PostDropNavState("recon") → 飞到侦察区 → ReconState
"""

import asyncio

from .base_state import BaseState, ExecutionResult
from .post_drop_nav import PostDropNavState
from .recon import ReconState


class DropState(BaseState):
    """舵机释放 → 判去向。"""

    def __init__(self, bottle_index: int, timeout_s: float = 10):
        super().__init__("Drop", timeout_s)
        self.bottle_index = bottle_index
        self._nav_done = False  # PostDropNav 是否已完成

    async def execute(self, interface):
        # ---- 首次调用：释放舵机 → 推 PostDropNav ----
        if not self._nav_done:
            await interface.set_actuator(self.bottle_index, 1.0)
            await asyncio.sleep(0.5)
            await interface.set_actuator(self.bottle_index, -1.0)
            print(f"[投放] 瓶子 {self.bottle_index}: 已释放")

            self._nav_done = True
            if self.bottle_index == 2:
                print("[投放] 两轮投放均完成 → 前往侦察区")
                return ExecutionResult(interrupt=PostDropNavState(
                    destination="recon"))
            else:
                print(f"[投放] 瓶子 {self.bottle_index} → 返回中断点继续搜索")
                return ExecutionResult(interrupt=PostDropNavState(
                    destination="return"))

        # ---- 导航完成：bottle1 → 回到 SearchState, bottle2 → 推 ReconState ----
        if self.bottle_index == 2 and not self.is_completed:
            self.is_completed = True
            print("[投放] 到达侦察区 → 切入侦察状态")
            return ExecutionResult(interrupt=ReconState())

        # ---- ReconState 完成后 resume → 退出 ----
        self.is_completed = True
        return ExecutionResult(done=True)
