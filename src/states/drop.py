"""投放状态 —— 仅控制舵机释放。对准已在 AlignState 中完成。"""

import asyncio

from .base_state import BaseState


class DropState(BaseState):
    """通过舵机释放载荷。此状态不包含视觉逻辑。"""

    def __init__(self, bottle_index: int, timeout_s: float = 10):
        super().__init__("Drop", timeout_s)
        self.bottle_index = bottle_index

    async def execute(self, interface):
        # 释放舵机
        await interface.set_actuator(self.bottle_index, 1.0)
        await asyncio.sleep(0.5)
        await interface.set_actuator(self.bottle_index, -1.0)

        print(f"[投放] 瓶子 {self.bottle_index}: 已释放")
        self.is_completed = True
        return True, None
