"""
Search state — coarse cylinder detection at cruise altitude.

Performs a single YOLO detection pass from 7 m to locate all cylinders,
classifies them (15/20/25 cm), selects two drop targets, and stores
the results in interface.shared for subsequent AlignState / DropState.
"""

from .base_state import BaseState
from config import CRUISE_ALTITUDE_M, DROP_ZONE_DISTANCE_M, YOLO_CONFIDENCE_THRESHOLD


class SearchState(BaseState):
    """Coarse detection — executed once at cruise altitude."""

    def __init__(self, timeout_s: float = 30):
        super().__init__("Search", timeout_s)

    async def enter(self, interface):
        await super().enter(interface)
        # Ensure we are at the drop-zone overhead position
        sp = interface.field_to_ned(DROP_ZONE_DISTANCE_M, 0.0, CRUISE_ALTITUDE_M)
        interface.update_setpoint(sp)

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "coarse search timeout"
            return True, None

        alt = await interface.get_altitude()
        if abs(alt - CRUISE_ALTITUDE_M) > 1.0:
            return False, None  # altitude not stabilized yet

        # ---- Single detection pass ----
        try:
            from vision.yolo_detector import get_detector
            detector = get_detector()
            frame = await _capture_frame_async()
            cylinders = detector.detect_cylinders(frame, alt)
        except Exception as e:
            self.error = f"detection failed: {e}"
            print(f"[Search] {self.error}")
            return False, None

        if len(cylinders) < 2:
            self.error = f"only {len(cylinders)} cylinders detected, need >= 2"
            print(f"[Search] {self.error}")
            return False, None

        target1, target2 = detector.select_targets(cylinders)

        interface.shared["drop_targets"] = (target1, target2)
        print(f"[Search] Coarse detection complete:")
        print(f"  Target 1: type={target1.cylinder_type}, "
              f"NED offset=({target1.ned_offset[0]:.2f}, {target1.ned_offset[1]:.2f}) m")
        print(f"  Target 2: type={target2.cylinder_type}, "
              f"NED offset=({target2.ned_offset[0]:.2f}, {target2.ned_offset[1]:.2f}) m")

        self.is_completed = True
        return True, None


async def _capture_frame_async():
    """Capture a single frame from the camera (runs in thread pool)."""
    import asyncio
    import concurrent.futures

    from vision.yolo_detector import _camera

    if _camera is None:
        raise RuntimeError("Camera not initialized — call init_camera() first")

    loop = asyncio.get_running_loop()
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    ret, frame = await loop.run_in_executor(executor, _camera.read)
    if not ret:
        raise RuntimeError("Failed to capture frame from camera")
    return frame
