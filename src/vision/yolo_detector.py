"""
基于 YOLO 的圆柱体检测器，用于投掷区。

工作流程：YOLO 粗略检测 → HoughCircles 精细定位 →
针孔模型 NED 偏移 → 圆柱体类型分类。
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .object_size_calculator import ObjectSizeCalculator


# 共享相机句柄 —— 初始化一次，在各状态间复用。
_camera: Optional[cv2.VideoCapture] = None


def init_camera(source=0, width=1280, height=720):
    """初始化全局相机句柄。"""
    global _camera
    _camera = cv2.VideoCapture(source)
    _camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    _camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not _camera.isOpened():
        raise RuntimeError(f"无法打开相机源 {source}")
    print(f"[视觉] 相机已打开: {source}, {width}x{height}")


def release_camera():
    """释放全局相机句柄。"""
    global _camera
    if _camera is not None:
        _camera.release()
        _camera = None


@dataclass
class Cylinder:
    """单个圆柱体的检测结果。"""
    bbox: Tuple[int, int, int, int]       # (x, y, w, h) 像素坐标
    center_uv: Tuple[float, float]         # 像素坐标中的中心点
    diameter_px: float                     # 检测到的像素直径
    estimated_diameter_cm: float           # 估算的实际直径（厘米）
    ned_offset: Tuple[float, float]        # NED 水平偏移（米）
    cylinder_type: Optional[int]           # 1=15厘米, 2=20厘米, 3=25厘米, None=不确定


class YOLODetector:
    """YOLO + HoughCircles 圆柱体检测器，带针孔模型定位。"""

    CYLINDER_DIAMETERS = {1: 0.15, 2: 0.20, 3: 0.25}

    def __init__(self, model_path: str, camera_matrix: np.ndarray,
                 dist_coeffs: Optional[np.ndarray] = None,
                 confidence_threshold: float = 0.5):
        self.confidence_threshold = confidence_threshold
        self.calc = ObjectSizeCalculator(camera_matrix, dist_coeffs)

        # 延迟加载 YOLO
        self._model_path = model_path
        self._yolo = None

    @property
    def yolo(self):
        if self._yolo is None:
            from ultralytics import YOLO
            self._yolo = YOLO(self._model_path)
        return self._yolo

    def detect_cylinders(self, frame: np.ndarray,
                         flight_altitude: float) -> List[Cylinder]:
        """
        单帧的完整检测流水线。

        参数:
            frame: 来自下视摄像头的 BGR 图像。
            flight_altitude: 离地高度（米）。

        返回:
            检测到的圆柱体列表，按估算直径排序（最小的在前）。
        """
        results = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 相机到圆柱体顶部的距离（圆柱体高度 = 0.30 米）
        camera_to_cylinder_z = flight_altitude - 0.30
        if camera_to_cylinder_z <= 0:
            return results

        yolo_results = self.yolo(frame, verbose=False)

        for box in yolo_results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            if conf < self.confidence_threshold:
                continue

            # HoughCircles 感兴趣区域
            margin = 10
            x1c = max(x1 - margin, 0)
            y1c = max(y1 - margin, 0)
            x2c = min(x2 + margin, frame.shape[1])
            y2c = min(y2 + margin, frame.shape[0])
            roi = gray[y1c:y2c, x1c:x2c]

            min_r, max_r = self._estimate_radius_range(flight_altitude)

            circles = cv2.HoughCircles(
                roi, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                param1=50, param2=30,
                minRadius=min_r, maxRadius=max_r,
            )

            if circles is None:
                continue

            circles = np.uint16(np.around(circles[0]))
            best = max(circles, key=lambda c: c[2])
            cx_roi, cy_roi, radius = best
            cx = cx_roi + x1c
            cy = cy_roi + y1c

            # 通过针孔模型计算实际直径
            real_w, _ = self.calc.compute_real_size(
                radius * 2, radius * 2, camera_to_cylinder_z
            )

            # NED 偏移
            offset_x, offset_y = self.calc.pixel_to_ned_offset(
                cx, cy, camera_to_cylinder_z
            )

            cyl_type = self._classify_cylinder(real_w, flight_altitude)

            cyl = Cylinder(
                bbox=(x1, y1, x2 - x1, y2 - y1),
                center_uv=(float(cx), float(cy)),
                diameter_px=float(radius * 2),
                estimated_diameter_cm=real_w * 100,
                ned_offset=(offset_x, offset_y),
                cylinder_type=cyl_type,
            )
            results.append(cyl)

        results.sort(key=lambda c: c.estimated_diameter_cm)
        return results

    def select_targets(self, cylinders: List[Cylinder]
                       ) -> Tuple[Cylinder, Cylinder]:
        """
        从检测到的圆柱体中选择两个投放目标。
        优先选择类型 1（15 厘米，500 分），然后类型 2（20 厘米，300 分）。
        """
        type1 = [c for c in cylinders if c.cylinder_type == 1]
        type2 = [c for c in cylinders if c.cylinder_type == 2]
        type3 = [c for c in cylinders if c.cylinder_type == 3]

        if type1:
            first = type1[0]
            second = type2[0] if type2 else (type3[0] if type3 else type1[0])
        elif type2:
            first = type2[0]
            second = type3[0] if type3 else type2[0]
        else:
            first = type3[0] if type3 else cylinders[0]
            second = cylinders[1] if len(cylinders) > 1 else first

        return first, second

    # ------------------------------------------------------------------
    # 辅助方法
    # ------------------------------------------------------------------

    def _estimate_radius_range(self, altitude: float) -> Tuple[int, int]:
        """根据高度估算 HoughCircles 的像素半径范围。"""
        # 15 厘米圆柱体: r_px = (0.15 * fx) / (2 * altitude)
        min_r = int((0.15 * self.calc.fx) / (2 * altitude) * 0.7)
        # 25 厘米圆柱体
        max_r = int((0.25 * self.calc.fx) / (2 * altitude) * 1.3)
        return max(min_r, 3), max(max_r, 5)

    def _classify_cylinder(self, estimated_diameter_m: float,
                           altitude: float) -> Optional[int]:
        """从估算直径分类圆柱体类型。"""
        d_cm = estimated_diameter_m * 100

        if altitude < 3.0:
            if d_cm < 17.5:
                return 1
            elif d_cm < 22.5:
                return 2
            else:
                return 3
        else:
            if d_cm < 18:
                return 1
            elif d_cm < 23:
                return 2
            elif d_cm > 22:
                return 3
            else:
                return None

    def cover_zone_check(self, flight_altitude: float,
                         zone_size: Tuple[float, float]) -> bool:
        """检查在给定高度下相机视场是否覆盖整个区域。"""
        import math
        h_fov = 2 * math.atan(self.calc.cx / self.calc.fx)
        v_fov = 2 * math.atan(self.calc.cy / self.calc.fy)
        h_coverage = 2 * flight_altitude * math.tan(h_fov)
        v_coverage = 2 * flight_altitude * math.tan(v_fov)
        return h_coverage >= zone_size[0] and v_coverage >= zone_size[1]


# ------------------------------------------------------------------
# 模块级单例，方便使用
# ------------------------------------------------------------------

_detector: Optional[YOLODetector] = None

# 默认相机矩阵（占位值 —— 比赛前需标定）
_DEFAULT_K = np.array([
    [800.0, 0.0, 640.0],
    [0.0, 800.0, 480.0],
    [0.0, 0.0, 1.0],
], dtype=np.float32)


def get_detector(model_path: str = "models/cylinder_yolov8n.pt",
                 camera_matrix: np.ndarray = None) -> YOLODetector:
    """获取或创建单例检测器实例。"""
    global _detector
    if _detector is None:
        if camera_matrix is None:
            camera_matrix = _DEFAULT_K
        _detector = YOLODetector(model_path, camera_matrix)
    return _detector
