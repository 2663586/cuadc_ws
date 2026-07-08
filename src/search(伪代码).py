# search.py — 伪代码

from states.base_state import BaseState, ExecutionResult
from states.align import AlignState
from config import (CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS,
                    ARRIVAL_THRESHOLD_M, EPSILON_DIAMETER_CM)

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
            self.error = "搜索超时"
            return ExecutionResult(done=True)

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
            if abs(diameter_cm - 15) <= EPSILON_DIAMETER_CM and self.goal[0] == 0:
                self.goal[0] = 1
                # 保存检测结果到共享缓存，供 AlignState 读取
                self._save_detection(interface, bottle=1, result=r)
                # 栈式抢占：挂起搜索 → 压入对准 → 对准完成后 resume 继续搜索
                return ExecutionResult(interrupt=AlignState(bottle_index=1))
            # 匹配 20cm 瓶 (goal[1])
            elif abs(diameter_cm - 20) <= EPSILON_DIAMETER_CM and self.goal[1] == 0:
                self.goal[1] = 1
                self._save_detection(interface, bottle=2, result=r)
                return ExecutionResult(interrupt=AlignState(bottle_index=2))

        if not arrived:
            return ExecutionResult()     # 还在路上，下一帧继续飞

        # ---- 无目标 → 推进到下一个航点，绕圈循环 ----
        self._wp_index = (self._wp_index + 1) % len(self._rect_waypoints)
        return ExecutionResult()

    # ------------------------------------------------------------------
    async def _fly_to_target(self, interface, wp_idx):
        """
        直接发送位置指令飞向航点 wp_idx。

        绕过 update_setpoint / 心跳，直接调用 MAVSDK offboard。
        （后续会修改心跳代码以配合此模式，避免 set_position_ned 交替冲突）
        """
        x, y, z, speed = self._rect_waypoints[wp_idx]
        target = interface.field_to_ned(x, y, z)

        # 直接发送位置 setpoint 到飞控
        await interface.drone.offboard.set_position_ned(target)

        # 到达判定
        pos = await interface.get_position_ned()
        dn = target.north_m - pos.north_m
        de = target.east_m - pos.east_m
        dist = (dn**2 + de**2) ** 0.5
        return dist < ARRIVAL_THRESHOLD_M

    # ------------------------------------------------------------------
    def _save_detection(self, interface, bottle: int, result: dict):
        """
        将检测结果写入 interface.shared，供 AlignState.enter() 读取。

        AlignState 期望 shared["drop_targets"] 是一个长度为 2 的 tuple，
        bottle=1 对应 index 0，bottle=2 对应 index 1。
        """
        # 构造 AlignState 能消费的目标对象（伪代码，实际需与 AlignState 对齐）
        target = TargetStub(
            ned_offset=(result["circle"].cx_px, result["circle"].cy_px),
            diameter_m=result["diameter_m"],
        )
        if "drop_targets" not in interface.shared:
            interface.shared["drop_targets"] = [None, None]
        interface.shared["drop_targets"][bottle - 1] = target
