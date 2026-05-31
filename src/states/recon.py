"""
Recon state — serpentine scan over the recon zone.

Flies a pre-computed rectangular scan pattern covering the 8×5 m recon area
at low altitude so the FPV video feed captures hazard-identification markers.
No onboard classification — the ground station crew reads the video feed.
"""

from .base_state import BaseState
from config import RECON_ZONE_DISTANCE_M, RECON_ALTITUDE_M, RECON_SCAN_STEP_M


class ReconState(BaseState):
    """Serpentine scan over the recon zone for video-based identification."""

    def __init__(self, timeout_s: float = 120):
        super().__init__("Recon", timeout_s)
        self._waypoints = []
        self._current_wp = 0

    async def enter(self, interface):
        await super().enter(interface)

        cx = RECON_ZONE_DISTANCE_M  # recon zone centre distance
        cy = 0.0                    # centre line
        half_w, half_h = 4.0, 2.5   # half-width, half-height of 8×5 m zone
        step = RECON_SCAN_STEP_M

        self._waypoints = [
            (cx,        cy + half_h, RECON_ALTITUDE_M),
            (cx,        cy - half_h, RECON_ALTITUDE_M),
            (cx + step, cy - half_h, RECON_ALTITUDE_M),
            (cx + step, cy + half_h, RECON_ALTITUDE_M),
            (cx + 2 * step, cy + half_h, RECON_ALTITUDE_M),
            (cx + 2 * step, cy - half_h, RECON_ALTITUDE_M),
            (cx + half_w, cy - half_h, RECON_ALTITUDE_M),
        ]
        self._current_wp = 0

        # Send first waypoint
        wp = self._waypoints[0]
        sp = interface.field_to_ned(*wp)
        interface.update_setpoint(sp)

        print(f"[Recon] Starting serpentine scan, "
              f"{len(self._waypoints)} waypoints")

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "recon timeout — partial scan usable"
            return True, None

        if self._current_wp >= len(self._waypoints):
            print("[Recon] Scan complete")
            self.is_completed = True
            return True, None

        wp_x, wp_y, wp_z = self._waypoints[self._current_wp]
        alt = await interface.get_altitude()

        # Estimate time per waypoint (~ 2-3 m spacing at ~3 m/s)
        dist_per_wp = 2.5
        est_time = dist_per_wp / 3.0

        if (self.elapsed() > (self._current_wp + 1) * est_time
                and abs(alt - wp_z) < 0.5):
            self._current_wp += 1
            if self._current_wp < len(self._waypoints):
                nx, ny, nz = self._waypoints[self._current_wp]
                sp = interface.field_to_ned(nx, ny, nz)
                interface.update_setpoint(sp)
                print(f"[Recon] Waypoint {self._current_wp}/{len(self._waypoints)}")

        return False, None
