"""
Align state — descend and visually servo to center over a target cylinder.

Reads the target from interface.shared (populated by SearchState).
Two sub-phases: descend to ~3 m, then visual-servo P-control to align.
"""

import asyncio

from .base_state import BaseState
from config import (
    DROP_ALIGN_ALTITUDE_M, DROP_ZONE_DISTANCE_M,
    ALIGN_THRESHOLD_M, SEARCH_TIMEOUT_S, VISUAL_SERVO_KP,
)


class AlignState(BaseState):
    """Fine alignment over a target cylinder via visual servoing."""

    def __init__(self, bottle_index: int, timeout_s: float = 60):
        super().__init__("Align", timeout_s)
        self.bottle_index = bottle_index

        # Sub-phase
        self.phase = "descend"  # descend → servo → done
        self._search_start = 0.0
        self._target = None

    async def enter(self, interface):
        await super().enter(interface)

        targets = interface.shared.get("drop_targets")
        if targets is None:
            self.error = "no coarse detection results in shared cache"
            print(f"[Align] {self.error}")
            return

        idx = 0 if self.bottle_index == 1 else 1
        self._target = targets[idx]

        if self._target is None:
            self.error = f"bottle {self.bottle_index} has no assigned target"
            print(f"[Align] {self.error}")
            return

        # Descend toward target cylinder
        sp = interface.field_to_ned(
            DROP_ZONE_DISTANCE_M + self._target.ned_offset[0],
            self._target.ned_offset[1],
            DROP_ALIGN_ALTITUDE_M,
        )
        interface.update_setpoint(sp)
        self.phase = "descend"
        self._search_start = self.elapsed()
        print(f"[Align] bottle {self.bottle_index}: descending to "
              f"{DROP_ALIGN_ALTITUDE_M:.1f} m above target")

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "align timeout"
            return True, None

        if self._target is None:
            return True, None  # enter() already set error

        if self.phase == "descend":
            return await self._do_descend(interface)
        elif self.phase == "servo":
            return await self._do_visual_servo(interface)
        return False, None

    async def _do_descend(self, interface):
        """Wait until altitude drops to ~3 m."""
        alt = await interface.get_altitude()
        if alt <= 3.0:
            self.phase = "servo"
            self._search_start = self.elapsed()
            print(f"[Align] bottle {self.bottle_index}: starting visual servo")
        return False, None

    async def _do_visual_servo(self, interface):
        """Visual servoing loop — detect cylinder, compute offset, P-control."""
        from config import YOLO_CONFIDENCE_THRESHOLD

        alt = await interface.get_altitude()

        try:
            from vision.yolo_detector import get_detector
            detector = get_detector()
            frame = await _capture_frame_async()
            cylinders = detector.detect_cylinders(frame, alt)
        except Exception as e:
            print(f"[Align] detection error: {e}")
            return False, None

        best = self._match_target(cylinders)

        if best is None:
            # Target lost — search timeout
            if self.elapsed() - self._search_start > SEARCH_TIMEOUT_S:
                print(f"[WARN] Align bottle {self.bottle_index}: "
                      f"target lost timeout, abandoning")
                return True, None

            # Hold position, drift toward last known location
            sp = interface.field_to_ned(
                DROP_ZONE_DISTANCE_M + self._target.ned_offset[0],
                self._target.ned_offset[1],
                alt,
            )
            interface.update_setpoint(sp)
            return False, None

        self._search_start = self.elapsed()  # reset search timer
        offset_x, offset_y = best.ned_offset
        self._target = best  # update with more precise low-altitude estimate

        # Check alignment
        if (abs(offset_x) < ALIGN_THRESHOLD_M
                and abs(offset_y) < ALIGN_THRESHOLD_M):
            interface.shared[f"bottle_{self.bottle_index}_aligned"] = True
            interface.shared[f"bottle_{self.bottle_index}_position"] = best
            self.is_completed = True
            print(f"[Align] bottle {self.bottle_index}: aligned")
            return True, None

        # P-control position adjustment
        sp = interface.field_to_ned(
            DROP_ZONE_DISTANCE_M + offset_x * VISUAL_SERVO_KP,
            offset_y * VISUAL_SERVO_KP,
            alt,
        )
        interface.update_setpoint(sp)
        return False, None

    def _match_target(self, cylinders: list):
        """Match detected cylinders to the target by nearest NED offset."""
        if not cylinders:
            return None
        tx, ty = self._target.ned_offset
        best = min(cylinders,
                   key=lambda c: (c.ned_offset[0] - tx) ** 2
                                 + (c.ned_offset[1] - ty) ** 2)
        return best


async def _capture_frame_async():
    """Capture a single frame from the camera (runs in thread pool)."""
    import asyncio
    import concurrent.futures

    from vision.yolo_detector import _camera

    if _camera is None:
        raise RuntimeError("Camera not initialized")

    loop = asyncio.get_running_loop()
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    ret, frame = await loop.run_in_executor(executor, _camera.read)
    if not ret:
        raise RuntimeError("Failed to capture frame")
    return frame
