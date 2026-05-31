"""Hover state — hold position for a fixed duration."""

from .base_state import BaseState


class HoverState(BaseState):
    """Hold position for a specified time. Heartbeat task maintains setpoint."""

    def __init__(self, hover_time: float = 1.0):
        super().__init__("Hover", timeout_s=hover_time + 5)
        self.hover_time = hover_time

    async def execute(self, interface):
        if self.elapsed() >= self.hover_time:
            self.is_completed = True
            return True, None
        return False, None
