"""
Pinhole-camera-model utilities: real-size estimation and pixel-to-NED conversion.

Based on the pinhole camera model:
    u = fx * Xc / Zc + cx
    v = fy * Yc / Zc + cy
"""

import numpy as np
from typing import Optional, Tuple


class ObjectSizeCalculator:
    """Compute real-world size and NED offset from pixel measurements."""

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
        Convert pixel dimensions to real-world metres.

        Args:
            pixel_width: object width in pixels.
            pixel_height: object height in pixels.
            distance_z: camera-to-object distance (m).

        Returns:
            (real_width_m, real_height_m)
        """
        real_width = (pixel_width * distance_z) / self.fx
        real_height = (pixel_height * distance_z) / self.fy
        return real_width, real_height

    def pixel_to_ned_offset(self, pixel_u: float, pixel_v: float,
                            distance_z: float) -> Tuple[float, float]:
        """
        Convert pixel coordinates to NED horizontal offset.

        Assumes downward-facing camera:
          - image up    = forward (North)
          - image right = right (East)

        Args:
            pixel_u, pixel_v: object centre in pixel coordinates.
            distance_z: camera-to-object distance (m).

        Returns:
            (offset_x_m, offset_y_m) — North/East offset from camera nadir.
        """
        du = pixel_u - self.cx
        dv = pixel_v - self.cy
        offset_x_m = (du * distance_z) / self.fx
        offset_y_m = (dv * distance_z) / self.fy
        return offset_x_m, offset_y_m

    def solve_pnp(self, image_points: np.ndarray,
                  object_points: np.ndarray) -> Optional[Tuple]:
        """
        Solve PnP for object pose relative to camera.

        Args:
            image_points: N×2 image feature coordinates.
            object_points: N×3 world coordinates of the same features.

        Returns:
            (rvec, tvec) or None on failure.
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
