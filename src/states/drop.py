"""Drop state — servo release only. Alignment is done in AlignState."""

import asyncio

from .base_state import BaseState


class DropState(BaseState):
    """Release a payload via servo actuator. No vision logic here."""

    def __init__(self, bottle_index: int, timeout_s: float = 10):
        super().__init__("Drop", timeout_s)
        self.bottle_index = bottle_index

    async def execute(self, interface):
        # Release servo
        await interface.set_actuator(self.bottle_index, 1.0)
        await asyncio.sleep(0.5)
        await interface.set_actuator(self.bottle_index, -1.0)

        print(f"[Drop] bottle {self.bottle_index}: released")
        self.is_completed = True
        return True, None
