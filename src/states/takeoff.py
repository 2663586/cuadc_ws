"""Takeoff state — climb to target altitude."""

from .base_state import BaseState


class TakeoffState(BaseState):
    """Climb to target altitude after arm + offboard."""

    def __init__(self, target_alt: float = 7.0, timeout_s: float = 30):
        super().__init__("Takeoff", timeout_s)
        self.target_alt = target_alt

    async def enter(self, interface):
        await super().enter(interface)
        # Set takeoff altitude and command takeoff
        sp = interface.field_to_ned(0.0, 0.0, self.target_alt)
        interface.update_setpoint(sp)
        await interface.takeoff(self.target_alt)

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "takeoff timeout"
            return True, None

        alt = await interface.get_altitude()
        if alt >= self.target_alt * 0.9:
            self.is_completed = True
            return True, None
        return False, None
