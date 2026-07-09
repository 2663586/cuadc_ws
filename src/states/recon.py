"""
侦察状态 —— 在侦察区域上方进行双线扫描。

在 5m 高度沿两条平行线覆盖 8×5 米的侦察区域，
使 FPV 视频流能够捕捉危险识别标记。
不进行机载分类 —— 地面站人员观看视频流进行判读。

航点坐标使用场地 NED 坐标系（north=场地前方, east=场地右方, up=高度）。
"""

from .base_state import BaseState

# 侦察区场地坐标参数
RECON_FORWARD_M = 59.0       # 侦察区中心前向距离 (m)
RECON_RIGHT_HALF = 3.0       # 侦察区半宽 (m), 留 1.0m 余量防止出界
RECON_ALTITUDE = 5.0         # 侦察扫描高度 (m)
RECON_LINE_SPACING = 3.0     # 两条扫描线的前向间距 (m)
RECON_SPEED_MPS = 3.0        # 扫描行进速度 (m/s)

RECON_POST_HOVER = 1.0       # 扫描结束后悬停稳定时间 (s)


class ReconState(BaseState):
    """在侦察区域上方进行双线扫描以进行视频识别。"""

    def __init__(self, timeout_s: float = 120):
        super().__init__("Recon", timeout_s)
        self._waypoints = []

        # 阶段控制: "transit" → "scan" → "stabilize"
        self._phase = "transit"
        self._phase_start = 0.0        # 当前阶段起始时间 (相对于 enter)

        # 扫描子状态
        self._current_wp = 0           # 当前正在飞往的航点索引
        self._seg_start = 0.0          # 当前航段起始时间

    async def enter(self, interface):
        await super().enter(interface)

        fwd = RECON_FORWARD_M
        half = RECON_RIGHT_HALF
        alt = RECON_ALTITUDE
        gap = RECON_LINE_SPACING

        # 两条平行扫描线
        # Line 1: 右扫 (forward=fwd,         right: -half → +half)
        # Line 2: 回扫 (forward=fwd - gap,   right: +half → -half)
        self._waypoints = [
            (-half, fwd,        alt),   # 0: 线1起点 (也是 transit 目标)
            (+half, fwd,        alt),   # 1: 线1终点
            (+half, fwd - gap,  alt),   # 2: 线2起点 (反向)
            (-half, fwd - gap,  alt),   # 3: 线2终点
        ]

        # 重置所有状态
        self._phase = "transit"
        self._phase_start = 0.0
        self._current_wp = 0
        self._seg_start = 0.0

        # 发布扫描起点的设定值, 无人机从当前位置飞过去
        wp0 = self._waypoints[0]
        sp = interface.field_to_ned(*wp0)
        interface.update_setpoint(sp)

        # 估算 transit 距离: 当前 NED → WP0 NED
        pos = await interface.get_position_ned()
        wp0_ned = interface.field_to_ned(*wp0)
        transit_dist = (
            (wp0_ned.north_m - pos.north_m) ** 2 +
            (wp0_ned.east_m - pos.east_m) ** 2
        ) ** 0.5
        self._transit_est = transit_dist / RECON_SPEED_MPS if RECON_SPEED_MPS > 0 else 10

        print(f"[侦察] 飞向扫描起点 ({-half:.0f},{fwd:.0f}), "
              f"距离约 {transit_dist:.1f}m, 预计 {self._transit_est:.1f}s, "
              f"线1: ({-half:.0f},{fwd:.0f}) -> ({half:.0f},{fwd:.0f}), "
              f"线2: ({half:.0f},{fwd-gap:.0f}) -> ({-half:.0f},{fwd-gap:.0f})")

    # ------------------------------------------------------------------
    # 航段距离
    # ------------------------------------------------------------------

    def _seg_distance(self, wp_index):
        """返回从上一航点到当前航点的距离 (m)。"""
        if wp_index == 0:
            return 0.0
        px, py, _ = self._waypoints[wp_index - 1]
        cx, cy, _ = self._waypoints[wp_index]
        return ((cx - px) ** 2 + (cy - py) ** 2) ** 0.5

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "侦察超时 —— 部分扫描结果仍可使用"
            return ExecutionResult(done=True)

        # ================================================================
        # 阶段 1: 飞向扫描起点
        # ================================================================
        if self._phase == "transit":
            wp0_x, wp0_y, wp_z = self._waypoints[0]
            alt = await interface.get_altitude()

            # 水平距离: 当前位置 → WP0
            pos = await interface.get_position_ned()
            wp0_ned = interface.field_to_ned(wp0_x, wp0_y, wp_z)
            h_dist = (
                (wp0_ned.north_m - pos.north_m) ** 2 +
                (wp0_ned.east_m  - pos.east_m)  ** 2
            ) ** 0.5

            # 到达判据: 水平距离 < 0.5m 且 高度接近目标
            # (时间作为兜底, 防止 odometry 漂移导致永远不满足)
            phase_elapsed = self.elapsed() - self._phase_start
            if h_dist < 0.5 and abs(alt - wp_z) < 0.5:
                print(f"[侦察] 到达起点 (距离 {h_dist:.1f}m), 开始扫描")
                self._phase = "scan"
                self._seg_start = self.elapsed()
            elif phase_elapsed > self._transit_est * 1.5:
                # 超时兜底: 1.5 倍预计时间后即使位置略差也强行开始
                print(f"[侦察] transit 超时 (距离 {h_dist:.1f}m), 强制开始扫描")
                self._phase = "scan"
                self._seg_start = self.elapsed()
            return False, None

        # ================================================================
        # 阶段 2: 航点扫描
        # ================================================================
        if self._phase == "scan":
            _, _, wp_z = self._waypoints[self._current_wp]  # 只用于高度判据
            alt = await interface.get_altitude()

            seg_dist = self._seg_distance(self._current_wp)
            est_time = seg_dist / RECON_SPEED_MPS if seg_dist > 0 else 0.5

            # 到达判据: 本段预计时间已过 且 高度接近目标
            if (self.elapsed() - self._seg_start > est_time
                    and abs(alt - wp_z) < 0.5):
                self._current_wp += 1

                if self._current_wp < len(self._waypoints):
                    # 推进到下一航点
                    nx, ny, nz = self._waypoints[self._current_wp]
                    sp = interface.field_to_ned(nx, ny, nz)
                    interface.update_setpoint(sp)
                    self._seg_start = self.elapsed()
                    print(f"[侦察] 航点 {self._current_wp}/{len(self._waypoints)}")
                else:
                    # 所有航点走完, 进入稳定阶段
                    print("[侦察] 扫描完成, 稳定 1s...")
                    self._phase = "stabilize"
                    self._phase_start = self.elapsed()
            return False, None

        # ================================================================
        # 阶段 3: 扫描后悬停稳定, 等待衔接 land
        # ================================================================
        if self._phase == "stabilize":
            if self.elapsed() - self._phase_start > RECON_POST_HOVER:
                print("[侦察] 稳定完成, 结束")
                self.is_completed = True
                return True, None
            return False, None

        return ExecutionResult()
