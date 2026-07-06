"""投放状态 —— 仅控制舵机释放。对准已在 AlignState 中完成。"""

import asyncio

from config import BOTTLE1_SERVO_INDEX, BOTTLE2_SERVO_INDEX
from .base_state import BaseState


class DropState(BaseState):
    """通过舵机释放载荷。

    servo_index 为 MavSDK set_actuator 编号：
      Pixhawk 4 FMU PWM 7 (AUX7) → 15  (瓶子1)
      Pixhawk 4 FMU PWM 8 (AUX8) → 16  (瓶子2)
    """

    def __init__(self, servo_index: int, timeout_s: float = 10):
        super().__init__("Drop", timeout_s)
        self.servo_index = servo_index

    async def execute(self, interface):
        # 释放舵机：正转 0.5s → 回位
        await interface.set_actuator(self.servo_index, 1.0)
        await asyncio.sleep(0.5)
        await interface.set_actuator(self.servo_index, -1.0)

        print(f"[投放] 舵机 AUX{self.servo_index - 8} (index={self.servo_index}): 已释放")
        self.is_completed = True
        return True, None
