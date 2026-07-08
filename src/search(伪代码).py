# search.py — 伪代码

import math

from states.base_state import BaseState, ExecutionResult
from states.align import AlignState
from config import (
    CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS,
    ARRIVAL_REL_THRESHOLD, BOTTLE_DIAMETER_TOLERANCE_CM,
    YOLO_CONFIDENCE_THRESHOLD, CIRCLE_CONF_THRESHOLD,
    YOLO_MODEL_PATH,
    SEARCH_RECT_HALF_N_M, SEARCH_RECT_HALF_E_M,
    SEARCH_RECT_CENTER_N_M, SEARCH_RECT_CENTER_E_M,
)

class SearchState(BaseState):

    def __init__(self, timeout_s=120):
        super().__init__("Search", timeout_s)
        self.goal = [0, 0]          # 0=未找到, 1=已找到
        self._rect_waypoints = []   # 矩形航线航点列表
        self._wp_index = 0

        # 视觉流水线: YOLO → Canny边缘 → HoughCircles → 直径
        from vision.pipeline import VisionPipeline
        self.pipeline = VisionPipeline(
            model_path=YOLO_MODEL_PATH,
            yolo_conf=YOLO_CONFIDENCE_THRESHOLD,
            circle_conf_threshold=CIRCLE_CONF_THRESHOLD,
        )

    # ------------------------------------------------------------------
    async def enter(self, interface):
        await super().enter(interface)

        # 矩形航线四顶点 —— 与投放区 (5×8) 同心的 3×6 矩形
        # 短边 (N, 3m) 沿飞行前方，长边 (E, 6m) 沿飞行右方
        # 顺时针遍历：SW → NW → NE → SE
        cN = SEARCH_RECT_CENTER_N_M
        cE = SEARCH_RECT_CENTER_E_M
        hN = SEARCH_RECT_HALF_N_M  # 1.5
        hE = SEARCH_RECT_HALF_E_M  # 3.0

        # 每个航点: (x, y, z, speed_mps)
        self._rect_waypoints = [
            (cN - hN, cE - hE, CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS),  # SW
            (cN + hN, cE - hE, CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS),  # NW
            (cN + hN, cE + hE, CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS),  # NE
            (cN - hN, cE + hE, CRUISE_ALTITUDE_M, SEARCH_SPEED_MPS),  # SE
        ]
        self._wp_index = 0

        # 预计算每段航段长度（用于到达判据中的相对误差）
        n = len(self._rect_waypoints)
        self._segment_lengths = []
        for i in range(n):
            curr = self._rect_waypoints[i]
            nxt = self._rect_waypoints[(i + 1) % n]
            self._segment_lengths.append(
                math.hypot(nxt[0] - curr[0], nxt[1] - curr[1])
            )

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
        frame = await _capture_frame_async()

        # VisionPipeline: YOLO → Canny边缘 → HoughCircles → 针孔模型算直径
        results = self.pipeline.process_frame(frame, alt_rel_m=alt)

        for r in results:
            if not r["edge_success"]:
                continue                # 圆检测失败，跳过

            diameter_cm = r["diameter_m"] * 100   # 真实直径 (cm)
            # 匹配 15cm 瓶 (goal[0])
            if abs(diameter_cm - 15) <= BOTTLE_DIAMETER_TOLERANCE_CM and self.goal[0] == 0:
                self.goal[0] = 1
                # 保存检测结果到共享缓存，供 AlignState 读取
                self._save_detection(interface, bottle=1, result=r)
                # 栈式抢占：挂起搜索 → 压入对准 → 对准完成后 resume 继续搜索
                return ExecutionResult(interrupt=AlignState(bottle_index=1))
            # 匹配 20cm 瓶 (goal[1])
            elif abs(diameter_cm - 20) <= BOTTLE_DIAMETER_TOLERANCE_CM and self.goal[1] == 0:
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
        通过心跳机制飞向航点 wp_idx。

        计算 NED 目标后调用 interface.update_setpoint() 更新共享
        setpoint，由后台心跳以固定频率发送至飞控。与所有其他状态
        使用相同的 setpoint 通道，无冲突。

        到达判据使用相对误差：
            剩余距离 < 航段长度 × ARRIVAL_REL_THRESHOLD
        即已飞过约 95% 航段即认为到达，避免绝对阈值在不同航段长度下
        过松或过紧的问题。
        """
        x, y, z, speed = self._rect_waypoints[wp_idx]
        target = interface.field_to_ned(x, y, z)

        # 更新共享 setpoint，由心跳循环以 OFFBOARD_HEARTBEAT_HZ 频率发送
        interface.update_setpoint(target)

        # 到达判定 —— 相对误差
        pos = await interface.get_position_ned()
        dn = target.north_m - pos.north_m
        de = target.east_m - pos.east_m
        dist = math.hypot(dn, de)
        seg_length = self._segment_lengths[wp_idx]
        return dist < seg_length * ARRIVAL_REL_THRESHOLD

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
