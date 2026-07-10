"""投放状态 —— 舵机释放 + goal 跟踪 + 去向判定。

对准已在 AlignPreciseState 中完成。
投放后更新 goal 跟踪，判定去向：
  - goal==[1,1] → PostDropNavState(destination="recon")
  - 否则        → PostDropNavState(destination="return")
"""

import asyncio

from .base_state import BaseState, ExecutionResult
from .post_drop_nav import PostDropNavState


class DropState(BaseState):
    """舵机释放 → 更新 goal → 判定去向。"""

    def __init__(self, bottle_index: int, timeout_s: float = 10):
        super().__init__("Drop", timeout_s)
        self.bottle_index = bottle_index

    async def execute(self, interface):
        # ---- PostDropNav 完成后 resume，直接退出 ----
        if self.is_completed:
            return ExecutionResult(done=True)

        # ---- 释放舵机 ----
        await interface.set_actuator(self.bottle_index, 1.0)
        await asyncio.sleep(0.5)
        await interface.set_actuator(self.bottle_index, -1.0)

        print(f"[投放] 瓶子 {self.bottle_index}: 已释放")

        # ---- 更新 goal 跟踪 ----
        goal = interface.shared.get("goal")
        if goal is None:
            goal = [0, 0]
            interface.shared["goal"] = goal
        bottle_key = f"bottle_{self.bottle_index}_position"
        if bottle_key in interface.shared:
            detected = interface.shared[bottle_key]
            if isinstance(detected, dict):
                dia = detected.get("diameter_m", 0)
            else:
                dia = getattr(detected, "diameter_m", 0)
            if abs(dia - 0.15) < 0.02:
                goal[0] = 1
                print("[记录] goal[0]=1 (15cm 圆筒完成)")
            elif abs(dia - 0.20) < 0.02:
                goal[1] = 1
                print("[记录] goal[1]=1 (20cm 圆筒完成)")
            else:
                print(f"[记录] 未知直径 {dia:.3f}m, 不更新 goal")
        else:
            goal[self.bottle_index - 1] = 1
            print(f"[记录] goal[{self.bottle_index - 1}]=1 "
                  f"(瓶子 {self.bottle_index} 投放, 无直径数据)")

        # ---- 判定去向 ----
        self.is_completed = True
        if goal == [1, 1]:
            print("[投放] 两轮投放均完成 → 前往侦察区")
            return ExecutionResult(interrupt=PostDropNavState(
                destination="recon"))
        else:
            print(f"[投放] goal={goal} → 返回中断点继续搜索")
            return ExecutionResult(interrupt=PostDropNavState(
                destination="return"))
