"""
针孔相机模型工具：实际尺寸估算和像素到 NED 的转换。

基于针孔相机模型：
    u = fx * Xc / Zc + cx
    v = fy * Yc / Zc + cy
"""

import numpy as np
from typing import Optional, Tuple


class ObjectSizeCalculator:
    """从像素测量值计算实际世界尺寸和 NED 偏移。"""

    def __init__(self, camera_matrix: np.ndarray,
                 dist_coeffs: Optional[np.ndarray] = None):
        self.K = camera_matrix
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]
        self.dist = dist_coeffs

    def compute_real_size(self, pixel_width: float, pixel_height: float,
                          distance_z: float) -> Tuple[float, float]:
        """
        将像素尺寸转换为实际世界米制单位。

        参数:
            pixel_width: 物体在像素中的宽度。
            pixel_height: 物体在像素中的高度。
            distance_z: 相机到物体的距离（米）。

        返回:
            (实际宽度_米, 实际高度_米)
        """
        real_width = (pixel_width * distance_z) / self.fx
        real_height = (pixel_height * distance_z) / self.fy
        return real_width, real_height

    def pixel_to_ned_offset(self, pixel_u: float, pixel_v: float,
                            distance_z: float) -> Tuple[float, float]:
        """
        将像素坐标转换为 NED 水平偏移。

        假设下视摄像头：
          - 图像上方    = 前方（北）
          - 图像右方    = 右方（东）

        参数:
            pixel_u, pixel_v: 物体在像素坐标中的中心点。
            distance_z: 相机到物体的距离（米）。

        返回:
            (offset_x_m, offset_y_m) —— 相对于相机天底点的北/东偏移。
        """
        du = pixel_u - self.cx
        dv = pixel_v - self.cy
        offset_x_m = (du * distance_z) / self.fx
        offset_y_m = (dv * distance_z) / self.fy
        return offset_x_m, offset_y_m

    def solve_pnp(self, image_points: np.ndarray,
                  object_points: np.ndarray) -> Optional[Tuple]:
        """
        通过 PnP 求解物体相对于相机的位姿。

        参数:
            image_points: N×2 图像特征坐标。
            object_points: 相同特征的 N×3 世界坐标。

        返回:
            (rvec, tvec) 或 None（求解失败时）。
        """
        success, rvec, tvec = cv2.solvePnP(
            object_points.astype(np.float32),
            image_points.astype(np.float32),
            self.K,
            self.dist if self.dist is not None else np.zeros((4, 1)),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            return None
        return rvec, tvec
