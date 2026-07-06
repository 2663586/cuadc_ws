"""
搜索状态 —— 在巡航高度进行圆柱体粗略检测。

从 7 米高度执行单次 YOLO 检测来定位所有圆柱体，
对其进行分类（15/20/25 厘米），选择两个投放目标，
并将结果存储在 interface.shared 中供后续 AlignState / DropState 使用。
"""

from .base_state import BaseState, ExecutionResult
from config import CRUISE_ALTITUDE_M, DROP_ZONE_DISTANCE_M, YOLO_CONFIDENCE_THRESHOLD


class SearchState(BaseState):
    """粗略检测 —— 在巡航高度执行一次。"""

    def __init__(self, timeout_s: float = 30):
        super().__init__("Search", timeout_s)

    async def enter(self, interface):
        await super().enter(interface)
        # 确保我们位于投掷区正上方
        sp = interface.field_to_ned(DROP_ZONE_DISTANCE_M, 0.0, CRUISE_ALTITUDE_M)
        interface.update_setpoint(sp)

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "粗略搜索超时"
            return ExecutionResult(done=True)

        alt = await interface.get_altitude()
        if abs(alt - CRUISE_ALTITUDE_M) > 1.0:
            return ExecutionResult()  # 高度尚未稳定

        # ---- 单次检测 ----
        try:
            from vision.yolo_detector import get_detector
            detector = get_detector()
            frame = await _capture_frame_async()
            cylinders = detector.detect_cylinders(frame, alt)
        except Exception as e:
            self.error = f"检测失败: {e}"
            print(f"[搜索] {self.error}")
            return ExecutionResult()

        if len(cylinders) < 2:
            self.error = f"仅检测到 {len(cylinders)} 个圆柱体，需要 >= 2"
            print(f"[搜索] {self.error}")
            return ExecutionResult()

        target1, target2 = detector.select_targets(cylinders)

        interface.shared["drop_targets"] = (target1, target2)
        print(f"[搜索] 粗略检测完成:")
        print(f"  目标 1: 类型={target1.cylinder_type}, "
              f"NED 偏移=({target1.ned_offset[0]:.2f}, {target1.ned_offset[1]:.2f}) 米")
        print(f"  目标 2: 类型={target2.cylinder_type}, "
              f"NED 偏移=({target2.ned_offset[0]:.2f}, {target2.ned_offset[1]:.2f}) 米")

        self.is_completed = True
        return ExecutionResult(done=True)


async def _capture_frame_async():
    """从相机捕获单帧图像（在线程池中运行）。"""
    import asyncio
    import concurrent.futures

    from vision.yolo_detector import _camera

    if _camera is None:
        raise RuntimeError("相机未初始化 — 请先调用 init_camera()")

    loop = asyncio.get_running_loop()
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    ret, frame = await loop.run_in_executor(executor, _camera.read)
    if not ret:
        raise RuntimeError("从相机捕获帧失败")
    return frame
