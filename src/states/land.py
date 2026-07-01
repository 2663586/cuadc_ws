"""
精准降落 —— 双策略。

策略 A（视觉）:  搜索 "H" 标记 → 视觉伺服对准 → 降落。
策略 B（GPS 回退）: 飞回原点 (0,0,0) → 缓慢下降，可选视觉微调。
"""

from .base_state import BaseState
from config import (
    LAND_START_ALTITUDE_M, LAND_SAFE_ALTITUDE_M,
    LAND_DESCEND_RATE_MPS, ALIGN_THRESHOLD_M,
)


class PrecisionLandState(BaseState):
    """精准降落，视觉优先 / GPS 回退双策略。"""

    def __init__(self, timeout_s: float = 60, use_vision: bool = True):
        super().__init__("Land", timeout_s)
        self.use_vision = use_vision

    async def enter(self, interface):
        await super().enter(interface)
        # 飞到降落点上方
        sp = interface.field_to_ned(0.0, 0.0, LAND_START_ALTITUDE_M)
        interface.update_setpoint(sp)
        print(f"[降落] 开始精准降落 "
              f"({'视觉模式' if self.use_vision else 'GPS 回退模式'})")

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
    # 策略 A —— 视觉伺服到 H 标记
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
            print("[警告] 未找到 H 标记，切换到 GPS 回退模式")
            self.use_vision = False
            return False, None

        offset_x, offset_y = h_offset

        if abs(offset_x) < ALIGN_THRESHOLD_M and abs(offset_y) < ALIGN_THRESHOLD_M:
            if alt <= 0.8:
                await interface.land()
                self.is_completed = True
                return True, None
            else:
                # 已对准但高度仍高 —— 下降
                sp = interface.field_to_ned(
                    0.0, 0.0, max(alt - LAND_DESCEND_RATE_MPS, 0.5)
                )
                interface.update_setpoint(sp)
        else:
            # 水平微调
            Kp = 0.3
            sp = interface.field_to_ned(
                offset_x * Kp, offset_y * Kp, alt
            )
            interface.update_setpoint(sp)

        return False, None

    # ------------------------------------------------------------------
    # 策略 B —— GPS / 家点回退
    # ------------------------------------------------------------------

    async def _gps_fallback_landing(self, interface):
        """
        飞回原点 (0, 0, 0)，然后缓慢下降。
        如果有相机帧可用，则使用视觉微调。
        """
        alt = await interface.get_altitude()

        if alt > LAND_SAFE_ALTITUDE_M:
            # 正常下降到安全高度
            sp = interface.field_to_ned(
                0.0, 0.0,
                max(alt - LAND_DESCEND_RATE_MPS, LAND_SAFE_ALTITUDE_M)
            )
            interface.update_setpoint(sp)
            return False, None

        # 低于安全高度 —— 缓慢下降，可选视觉微调
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
    桩代码：检测 'H' 降落标记并返回 (offset_x_m, offset_y_m)。

    替换为实际的 H 检测（模板匹配 / 简单斑点 / 轮廓）。
    未找到时返回 None。
    """
    import cv2
    import numpy as np

    # 简单的圆形检测作为占位 —— 替换为实际的 H 检测
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
        param1=50, param2=30, minRadius=15, maxRadius=80
    )

    if circles is None:
        return None

    # 假设最大的圆就是 H 标记（80 厘米直径的圆）
    circles = np.uint16(np.around(circles[0]))
    best = max(circles, key=lambda c: c[2])
    cx, cy, r = best

    # 使用已知标记尺寸将像素偏移转换为米
    # 80 厘米圆 → 像素直径 = 2*r
    real_diameter_m = 0.80
    pixel_size_m = real_diameter_m / (2 * r)

    h, w = frame.shape[:2]
    offset_x_px = cx - w / 2
    offset_y_px = cy - h / 2

    offset_x_m = offset_x_px * pixel_size_m
    offset_y_m = offset_y_px * pixel_size_m

    return offset_x_m, offset_y_m
