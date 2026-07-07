# search.py — 伪代码

class SearchState(BaseState):

    def __init__(self, timeout_s=120):
        super().__init__("Search", timeout_s)
        self.goal = [0, 0]          # 0=未找到, 1=已找到
        self._rect_waypoints = []   # 矩形航线航点列表
        self._wp_index = 0

        # 视觉流水线: YOLO → Canny边缘 → HoughCircles → 直径
        from vision.pipeline import VisionPipeline
        self.pipeline = VisionPipeline(
            model_path="models/yolov11n_800_best_FP16.engine",
            yolo_conf=0.5,
            circle_conf_threshold=0.3,
        )

    # ------------------------------------------------------------------
    async def enter(self, interface):
        await super().enter(interface)

        # 矩形航线四顶点（坐标 + 速度，均已预先给定）
        # 每个航点: (x, y, z, speed_mps)
        self._rect_waypoints = [
            (P1_x, P1_y, CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS),
            (P2_x, P2_y, CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS),
            (P3_x, P3_y, CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS),
            (P4_x, P4_y, CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS),
        ]
        self._wp_index = 0

    # ------------------------------------------------------------------
    async def execute(self, interface):
        # ---- 超时 ----
        if self.is_timed_out():
            return True, None

        # ---- 飞到当前航点（速度控制） ----
        arrived = await self._fly_to_target(interface, self._wp_index)

        # ---- 执行检测 ----
        alt = await interface.get_altitude()
        frame = await _capture_frame()

        # VisionPipeline: YOLO → Canny边缘 → HoughCircles → 针孔模型算直径
        results = self.pipeline.process_frame(frame, alt_rel_m=alt)

        for r in results:
            if not r["edge_success"]:
                continue                # 圆检测失败，跳过

            diameter_cm = r["diameter_m"] * 100   # 真实直径 (cm)
            # 匹配 15cm 瓶 (goal[0])
            if abs(diameter_cm - 15) <= epsilon and self.goal[0] == 0:
                self.goal[0] = 1
                switch_to_align(bottle=1, target=r)  # 切换至 Align（待实现）
                return True, None
            # 匹配 20cm 瓶 (goal[1])
            elif abs(diameter_cm - 20) <= epsilon and self.goal[1] == 0:
                self.goal[1] = 1
                switch_to_align(bottle=2, target=r)  # 切换至 Align（待实现）
                return True, None
        if not arrived:
            return False, None          # 还在路上，下一帧继续飞
        # ---- 无目标 → 推进到下一个航点，绕圈循环 ----
        self._wp_index = (self._wp_index + 1) % len(self._rect_waypoints)
        return False, None

    # ------------------------------------------------------------------
    async def _fly_to_target(self, interface, wp_idx):
        """
        以指定速度飞向航点 wp_idx。

        每帧从 _rect_waypoints 读目标坐标+速度，计算方向向量，
        用 set_velocity_ned 驱动。到达后切回位置保持。
        """
        x, y, z, speed = self._rect_waypoints[wp_idx]
        target = interface.field_to_ned(x, y, z)

        pos = await interface.get_position_ned()
        dn = target.north_m - pos.north_m
        de = target.east_m - pos.east_m
        dist = (dn**2 + de**2) ** 0.5

        if dist < ARRIVAL_THRESHOLD_M:
            interface.update_setpoint(target)
            return True

        vn = (dn / dist) * speed
        ve = (de / dist) * speed

        from mavsdk.offboard import VelocityNedYaw
        await interface.drone.offboard.set_velocity_ned(
            VelocityNedYaw(vn, ve, 0.0, target.yaw_deg))
        return False
