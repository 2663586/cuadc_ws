"""
Precision landing — dual-strategy.

Strategy A (vision):  search for "H" marker → visual servo align → land.
Strategy B (GPS fallback): fly to home (0,0,0) → slow descent with optional visual trim.
"""

from .base_state import BaseState
from config import (
    LAND_START_ALTITUDE_M, LAND_SAFE_ALTITUDE_M,
    LAND_DESCEND_RATE_MPS, ALIGN_THRESHOLD_M,
)


class PrecisionLandState(BaseState):
    """Precision landing with vision-primary / GPS-fallback dual strategy."""

    def __init__(self, timeout_s: float = 60, use_vision: bool = True):
        super().__init__("Land", timeout_s)
        self.use_vision = use_vision

    async def enter(self, interface):
        await super().enter(interface)
        # Fly to above the landing pad
        sp = interface.field_to_ned(0.0, 0.0, LAND_START_ALTITUDE_M)
        interface.update_setpoint(sp)
        print(f"[Land] Starting precision landing "
              f"({'vision' if self.use_vision else 'GPS fallback'})")

    async def execute(self, interface):
        if self.is_timed_out():
            await interface.land()
            self.is_completed = True
            return True, None

        if self.use_vision:
            return await self._vision_landing(interface)
        else:
            return await self._gps_fallback_landing(interface)

    # ------------------------------------------------------------------
    # Strategy A — visual servo to H marker
    # ------------------------------------------------------------------

    async def _vision_landing(self, interface):
        alt = await interface.get_altitude()

        try:
            from vision.yolo_detector import _camera
            import asyncio
            import concurrent.futures

            if _camera is not None:
                loop = asyncio.get_running_loop()
                executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
                ret, frame = await loop.run_in_executor(executor, _camera.read)
                if ret:
                    h_offset = _detect_h_marker(frame)
                else:
                    h_offset = None
            else:
                h_offset = None
        except Exception:
            h_offset = None

        if h_offset is None:
            print("[WARN] H marker not found, switching to GPS fallback")
            self.use_vision = False
            return False, None

        offset_x, offset_y = h_offset

        if abs(offset_x) < ALIGN_THRESHOLD_M and abs(offset_y) < ALIGN_THRESHOLD_M:
            if alt <= 0.8:
                await interface.land()
                self.is_completed = True
                return True, None
            else:
                # Aligned but still high — descend
                sp = interface.field_to_ned(
                    0.0, 0.0, max(alt - LAND_DESCEND_RATE_MPS, 0.5)
                )
                interface.update_setpoint(sp)
        else:
            # Horizontal trim
            Kp = 0.3
            sp = interface.field_to_ned(
                offset_x * Kp, offset_y * Kp, alt
            )
            interface.update_setpoint(sp)

        return False, None

    # ------------------------------------------------------------------
    # Strategy B — GPS / home-point fallback
    # ------------------------------------------------------------------

    async def _gps_fallback_landing(self, interface):
        """
        Fly to home (0, 0, 0), then descend slowly.
        Uses visual trim if a camera frame is available.
        """
        alt = await interface.get_altitude()

        if alt > LAND_SAFE_ALTITUDE_M:
            # Normal descent to safe altitude
            sp = interface.field_to_ned(
                0.0, 0.0,
                max(alt - LAND_DESCEND_RATE_MPS, LAND_SAFE_ALTITUDE_M)
            )
            interface.update_setpoint(sp)
            return False, None

        # Below safe altitude — slow descent with optional visual trim
        h_offset = None
        try:
            from vision.yolo_detector import _camera
            import asyncio
            import concurrent.futures

            if _camera is not None:
                loop = asyncio.get_running_loop()
                executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
                ret, frame = await loop.run_in_executor(executor, _camera.read)
                if ret:
                    h_offset = _detect_h_marker(frame)
        except Exception:
            pass

        if h_offset is not None:
            offset_x, offset_y = h_offset
            dist = (offset_x ** 2 + offset_y ** 2) ** 0.5
            rate = max(0.1, LAND_DESCEND_RATE_MPS * (1.0 - dist / 2.0))
            sp = interface.field_to_ned(
                offset_x * 0.2, offset_y * 0.2,
                max(alt - rate, 0.3),
            )
            interface.update_setpoint(sp)
        else:
            sp = interface.field_to_ned(0.0, 0.0, max(alt - 0.1, 0.3))
            interface.update_setpoint(sp)

        if alt <= 0.5:
            await interface.land()
            self.is_completed = True
            return True, None

        return False, None


def _detect_h_marker(frame):
    """
    Stub: detect the 'H' landing marker and return (offset_x_m, offset_y_m).

    Replace with actual H-detection (template matching / simple blob / contour).
    Returns None if not found.
    """
    import cv2
    import numpy as np

    # Simple circle detection as placeholder — replace with real H detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
        param1=50, param2=30, minRadius=15, maxRadius=80
    )

    if circles is None:
        return None

    # Assume largest circle is the H marker (80 cm diameter circle)
    circles = np.uint16(np.around(circles[0]))
    best = max(circles, key=lambda c: c[2])
    cx, cy, r = best

    # Convert pixel offset to meters using known marker size
    # 80 cm circle → pixel diameter = 2*r
    real_diameter_m = 0.80
    pixel_size_m = real_diameter_m / (2 * r)

    h, w = frame.shape[:2]
    offset_x_px = cx - w / 2
    offset_y_px = cy - h / 2

    offset_x_m = offset_x_px * pixel_size_m
    offset_y_m = offset_y_px * pixel_size_m

    return offset_x_m, offset_y_m
