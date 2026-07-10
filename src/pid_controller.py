"""
PID 控制器 —— 像素误差 → 速度指令。

AlignPreciseState 在精细对准阶段调用:
    v_north, v_east = pid.update(dx_px, dy_px, dt)
"""


class PIDController:
    """像素空间 PID 速度控制器。

    输入: 圆心与图像中心的像素差 (dx_px, dy_px) 和时间步长 dt
    输出: NED 速度指令 (v_north_mps, v_east_mps)
    """

    def __init__(self, kp: float = 2.0, ki: float = 0.0, kd: float = 0.0,
                 max_vel: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_vel = max_vel

        # 积分累加
        self._integral_x = 0.0
        self._integral_y = 0.0

        # 上一帧误差（用于微分）
        self._prev_dx = 0.0
        self._prev_dy = 0.0
        self._initialized = False

    def reset(self):
        """重置积分和微分状态。"""
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_dx = 0.0
        self._prev_dy = 0.0
        self._initialized = False

    def update(self, dx_px: float, dy_px: float, dt: float
               ) -> tuple[float, float]:
        """单步更新，返回 (v_north_mps, v_east_mps)。

        Args:
            dx_px: 圆心 x 像素 - 图像中心 x (右=东)
            dy_px: 圆心 y 像素 - 图像中心 y (下=南, 即 -北)
            dt:    距上次调用的时间间隔 (秒)
        """
        # TODO: 替换为你自己的 PID 实现
        # ---- 占位: 简单 P 控制 ----
        v_east = self.kp * dx_px * 0.001   # 粗略 px→m 缩放
        v_north = -self.kp * dy_px * 0.001  # 图像下=场地南=-北

        # 限速
        speed = (v_north ** 2 + v_east ** 2) ** 0.5
        if speed > self.max_vel:
            scale = self.max_vel / speed
            v_north *= scale
            v_east *= scale

        self._prev_dx = dx_px
        self._prev_dy = dy_px
        self._initialized = True

        return v_north, v_east
