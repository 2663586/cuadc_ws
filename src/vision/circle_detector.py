#!/usr/bin/env python3
"""
圆检测模块 — Canny 边缘 + HoughCircles + 针孔相机模型
========================================================
从 YOLO 检测框 ROI 中提取圆形边缘、检测圆心与半径、计算真实直径。

用法:
    from vision.circle_detector import CircleDetector

    cd = CircleDetector(camera_matrix, dist_coeffs)
    result = cd.detect(frame, bbox)        # → CircleResult | None
    diameter_m = cd.compute_diameter(radius_px, alt_rel_m)

参考:
    circle_detect_pipeline.py — 原始离线处理脚本
    针孔相机模型与图像中物体大小计算.md — 理论推导
"""

import cv2
import numpy as np
from typing import Optional, Tuple, NamedTuple
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------


class CircleResult(NamedTuple):
    """单次圆检测的结果"""

    cx_px: int  # 圆心 x (全帧像素坐标)
    cy_px: int  # 圆心 y (全帧像素坐标)
    radius_px: int  # 像素半径
    diameter_px: float  # 像素直径 (= 2 × radius_px)
    diameter_m: float  # 估算真实直径 (米), -1.0 表示无法计算
    bbox: Tuple[int, int, int, int]  # 使用的检测框 (x1, y1, x2, y2)
    roi_offset: Tuple[int, int]  # ROI 在全帧中的偏移 (roi_x1, roi_y1)


# ---------------------------------------------------------------------------
# 默认相机内参 — Orin NX 下视摄像头, 1280×720
# 棋盘格标定, 平均重投影误差 0.1784 px
# ---------------------------------------------------------------------------

DEFAULT_CAMERA_MATRIX = np.array(
    [
        [1059.1542, 0.0000, 638.3737],
        [0.0000, 1058.8622, 355.4191],
        [0.0000, 0.0000, 1.0000],
    ],
    dtype=np.float64,
)

DEFAULT_DIST_COEFFS = np.array(
    [0.034250, -0.037603, 0.003390, 0.002477, -0.017466],
    dtype=np.float64,
)

# ---------------------------------------------------------------------------
# 圆检测器
# ---------------------------------------------------------------------------


class CircleDetector:
    """在 YOLO 检测框 ROI 内进行 Canny+HoughCircles 圆检测"""

    def __init__(
        self,
        camera_matrix: np.ndarray = DEFAULT_CAMERA_MATRIX,
        dist_coeffs: np.ndarray = DEFAULT_DIST_COEFFS,
        bucket_height_m: float = 0.30,
        # ROI 参数
        roi_padding_ratio: float = 0.20,
        # Canny 参数
        canny_low: int = 40,
        canny_high: int = 120,
        # HoughCircles 参数
        hough_dp: float = 1.2,
        hough_min_dist: int = 20,
        hough_param1: int = 50,
        hough_param2: int = 25,
        hough_min_radius: int = 5,
        hough_max_radius: int = 100,
        # 可视化
        edge_color: Tuple[int, int, int] = (0, 255, 0),
        circle_color: Tuple[int, int, int] = (0, 0, 255),
        center_color: Tuple[int, int, int] = (0, 0, 255),
        text_color: Tuple[int, int, int] = (255, 255, 255),
        overlay_alpha: float = 0.5,
    ):
        """
        Args:
            camera_matrix:    相机内参矩阵 (3×3)
            dist_coeffs:      畸变系数 (k1,k2,p1,p2,k3)
            bucket_height_m:  桶的物理高度 (米), 用于修正相机到桶顶的距离
            roi_padding_ratio: ROI 扩展比例 (在 YOLO bbox 基础上向外扩展)
            canny_low/high:   Canny 边缘检测阈值
            hough_dp:         HoughCircles 累加器分辨率反比
            hough_min_dist:   检测圆之间的最小距离 (像素)
            hough_param1:     HoughCircles 内部 Canny 高阈值
            hough_param2:     累加器阈值 (越小检测越多假圆)
            hough_min_radius: 最小圆半径 (像素)
            hough_max_radius: 最大圆半径 (像素)
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.fx = float(camera_matrix[0, 0])
        self.fy = float(camera_matrix[1, 1])
        self.cx = float(camera_matrix[0, 2])
        self.cy = float(camera_matrix[1, 2])
        self.bucket_height_m = bucket_height_m

        self.roi_padding_ratio = roi_padding_ratio
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.hough_dp = hough_dp
        self.hough_min_dist = hough_min_dist
        self.hough_param1 = hough_param1
        self.hough_param2 = hough_param2
        self.hough_min_radius = hough_min_radius
        self.hough_max_radius = hough_max_radius

        # 可视化参数
        self.edge_color = edge_color
        self.circle_color = circle_color
        self.center_color = center_color
        self.text_color = text_color
        self.overlay_alpha = overlay_alpha

    # ------------------------------------------------------------------
    # 预计算畸变校正映射
    # ------------------------------------------------------------------

    def init_undistort_maps(self, width: int, height: int):
        """
        预计算畸变校正的 remap 映射表 (只需调用一次).

        Returns:
            (map1, map2) — 可直接传给 cv2.remap()
        """
        map1, map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            None,
            self.camera_matrix,
            (width, height),
            cv2.CV_16SC2,
        )
        return map1, map2

    def undistort(self, frame: np.ndarray, map1, map2) -> np.ndarray:
        """对单帧做畸变校正"""
        return cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

    # ------------------------------------------------------------------
    # 核心检测
    # ------------------------------------------------------------------

    def detect(
        self, frame: np.ndarray, bbox: Tuple[int, int, int, int]
    ) -> Optional[CircleResult]:
        """
        在 YOLO 检测框 ROI 内检测圆 (Canny 边缘 + HoughCircles)。

        Args:
            frame: 全帧图像 (BGR, H×W×3)
            bbox:  边界框 (x1, y1, x2, y2) — 像素坐标

        Returns:
            CircleResult — 检测到的圆的信息 (全帧坐标)
            None — 未检测到圆
        """
        x1, y1, x2, y2 = bbox

        # 1. 提取 ROI (带 padding)
        roi, roi_x1, roi_y1 = self._extract_roi(frame, x1, y1, x2, y2)
        if roi.size == 0:
            return None

        # 2. 圆检测
        circle_roi = self._detect_circle_in_roi(roi)
        if circle_roi is None:
            return None

        cx_roi, cy_roi, radius = circle_roi

        # 3. 转换到全帧坐标
        cx_full = roi_x1 + cx_roi
        cy_full = roi_y1 + cy_roi

        return CircleResult(
            cx_px=cx_full,
            cy_px=cy_full,
            radius_px=radius,
            diameter_px=2.0 * radius,
            diameter_m=-1.0,  # 调用方用 compute_diameter() 填充
            bbox=(x1, y1, x2, y2),
            roi_offset=(roi_x1, roi_y1),
        )

    def _extract_roi(
        self, frame: np.ndarray, x1: int, y1: int, x2: int, y2: int
    ) -> Tuple[np.ndarray, int, int]:
        """从帧中提取 ROI (带 padding). 返回 (roi, roi_x1, roi_y1)."""
        h, w = frame.shape[:2]
        bw, bh = x2 - x1, y2 - y1
        pad_w = int(bw * self.roi_padding_ratio)
        pad_h = int(bh * self.roi_padding_ratio)

        rx1 = max(0, x1 - pad_w)
        ry1 = max(0, y1 - pad_h)
        rx2 = min(w, x2 + pad_w)
        ry2 = min(h, y2 + pad_h)

        if rx2 <= rx1 or ry2 <= ry1:
            rx1, ry1, rx2, ry2 = x1, y1, x2, y2

        roi = frame[ry1:ry2, rx1:rx2]
        return roi, rx1, ry1

    def _detect_circle_in_roi(
        self, roi: np.ndarray
    ) -> Optional[Tuple[int, int, int]]:
        """
        在 ROI 灰度图中检测圆 (Canny + HoughCircles)。

        Returns:
            (cx_roi, cy_roi, radius) — ROI 内的坐标, 或 None
        """
        if roi.size == 0 or roi.shape[0] < 10 or roi.shape[1] < 10:
            return None

        # 灰度化
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi.copy()

        # 高斯模糊降噪
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 限制半径范围
        min_r = max(self.hough_min_radius, 3)
        max_r = min(
            self.hough_max_radius, min(roi.shape[0], roi.shape[1]) // 2
        )
        if max_r < min_r:
            return None

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_min_dist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=min_r,
            maxRadius=max_r,
        )

        if circles is None or len(circles[0]) == 0:
            return None

        # 选择半径最大的圆 (通常最可靠)
        circles = np.uint16(np.around(circles[0]))
        best = max(circles, key=lambda c: c[2])
        return int(best[0]), int(best[1]), int(best[2])

    # ------------------------------------------------------------------
    # 直径计算 (针孔相机模型)
    # ------------------------------------------------------------------

    def compute_diameter(self, radius_px: int, alt_rel_m: float) -> float:
        """
        基于针孔相机模型计算圆的真实直径。

        公式: D_real = (2 × r_px × Z_C) / fx

        其中:
          Z_C = alt_rel_m - bucket_height_m (相机到桶顶的距离)
          r_px = 像素半径
          fx   = 焦距 (像素)

        Args:
            radius_px:  圆的像素半径
            alt_rel_m:  相对起飞点高度 (米)

        Returns:
            真实直径 (米), 如果高度无效则返回 -1.0
        """
        z_c = alt_rel_m - self.bucket_height_m
        if z_c <= 0.0:
            return -1.0
        diameter_px = 2.0 * radius_px
        return (diameter_px * z_c) / self.fx

    # ------------------------------------------------------------------
    # Canny 边缘 (用于可视化)
    # ------------------------------------------------------------------

    def get_edges(self, roi: np.ndarray) -> np.ndarray:
        """
        对 ROI 执行 Canny 边缘检测, 返回二值边缘图。

        Args:
            roi: ROI 图像 (BGR 或灰度)

        Returns:
            二值边缘图 (与 ROI 同尺寸)
        """
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi.copy()

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        return edges

    # ------------------------------------------------------------------
    # 可视化叠加
    # ------------------------------------------------------------------

    def overlay(
        self,
        frame: np.ndarray,
        roi_x1: int,
        roi_y1: int,
        edges: np.ndarray,
        circle_result: Optional[CircleResult],
    ) -> np.ndarray:
        """
        在帧上叠加 Canny 边缘 (半透明绿色) 和检测圆/圆心 (红色)。

        Args:
            frame:         原始帧 (BGR)
            roi_x1, roi_y1: ROI 在全帧中的左上角坐标
            edges:         Canny 边缘二值图 (与 ROI 同尺寸)
            circle_result: 检测结果 (含坐标和直径信息), 或 None

        Returns:
            叠加后的帧 (新副本)
        """
        result = frame.copy()
        h_roi, w_roi = edges.shape[:2]
        roi_y2 = min(roi_y1 + h_roi, result.shape[0])
        roi_x2 = min(roi_x1 + w_roi, result.shape[1])
        h_roi = roi_y2 - roi_y1
        w_roi = roi_x2 - roi_x1

        if h_roi <= 0 or w_roi <= 0:
            return result

        # Canny 边缘叠加
        edges_cropped = edges[:h_roi, :w_roi]
        roi_region = result[roi_y1:roi_y2, roi_x1:roi_x2]
        green_overlay = np.zeros_like(roi_region)
        green_overlay[edges_cropped > 0] = self.edge_color
        blended = cv2.addWeighted(
            roi_region, 1.0 - self.overlay_alpha,
            green_overlay, self.overlay_alpha, 0,
        )
        result[roi_y1:roi_y2, roi_x1:roi_x2] = blended

        # 圆/圆心/标注
        if circle_result is not None:
            cx = circle_result.cx_px
            cy = circle_result.cy_px
            radius = circle_result.radius_px

            cx = int(np.clip(cx, 0, result.shape[1] - 1))
            cy = int(np.clip(cy, 0, result.shape[0] - 1))

            cv2.circle(result, (cx, cy), radius, self.circle_color, 2)
            cv2.circle(result, (cx, cy), 3, self.center_color, -1)

            cross = max(radius // 2, 3) if radius > 6 else 3
            cv2.line(
                result, (cx - cross, cy), (cx + cross, cy),
                self.center_color, 1,
            )
            cv2.line(
                result, (cx, cy - cross), (cx, cy + cross),
                self.center_color, 1,
            )

            if circle_result.diameter_m > 0:
                label = f"D={circle_result.diameter_m * 100:.1f}cm"
            else:
                label = f"r={radius}px"
            cv2.putText(
                result, label, (cx + radius + 5, cy - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.text_color, 1,
            )

        return result
