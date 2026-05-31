"""Transit state — fly to a field-coordinate waypoint at cruise speed."""

from typing import Optional

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState


class TransitState(BaseState):
    """
    Fly to target field coordinates (forward, right, height) at cruise speed.

    The setpoint is issued once in enter() and the heartbeat task maintains it.
    Completion is determined by elapsed time estimate + altitude proximity.
    """

    def __init__(self, x: float, y: float, z: float,
                 speed: float = 5.0, timeout_s: float = 60):
        super().__init__("Transit", timeout_s)
        self.target_x, self.target_y, self.target_z = x, y, z
        self.speed = speed

    async def enter(self, interface):
        await super().enter(interface)
        target_sp = interface.field_to_ned(self.target_x, self.target_y,
                                            self.target_z)
        interface.update_setpoint(target_sp)
        print(f"[Transit] -> field({self.target_x:.1f}, {self.target_y:.1f}, "
              f"{self.target_z:.1f}) @ {self.speed:.1f} m/s")

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "transit timeout"
            return True, None

        alt = await interface.get_altitude()
        # Estimate travel time from distance
        dist = (self.target_x ** 2 + self.target_y ** 2) ** 0.5
        est_time = dist / self.speed if self.speed > 0 else 10

        if self.elapsed() > est_time and abs(alt - self.target_z) < 0.5:
            self.is_completed = True
            return True, None
        return False, None
