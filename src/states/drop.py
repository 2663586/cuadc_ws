"""投放状态 —— 舵机释放 + goal 跟踪。

对准已在 AlignPreciseState 中完成。
投放后更新 goal 跟踪（15cm/20cm），返回 done=True。
"""

import asyncio

from .base_state import BaseState, ExecutionResult


class DropState(BaseState):
    """通过舵机释放载荷，更新 goal 跟踪。"""

    def __init__(self, bottle_index: int, timeout_s: float = 10):
        super().__init__("Drop", timeout_s)
        self.bottle_index = bottle_index

    async def execute(self, interface):
        # ---- 释放舵机 ----
        await interface.set_actuator(self.bottle_index, 1.0)
        await asyncio.sleep(0.5)
        await interface.set_actuator(self.bottle_index, -1.0)

        print(f"[投放] 瓶子 {self.bottle_index}: 已释放")

        # ---- 根据检测到的直径更新 goal 跟踪 ----
        goal = interface.shared.setdefault("goal", [0, 0])
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
            # 无直径数据，根据 bottle_index 推定
            goal[self.bottle_index - 1] = 1
            print(f"[记录] goal[{self.bottle_index - 1}]=1 "
                  f"(瓶子 {self.bottle_index} 投放, 无直径数据)")

        self.is_completed = True
        return ExecutionResult(done=True)
