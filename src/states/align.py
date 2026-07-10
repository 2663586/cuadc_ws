"""
粗对准状态 —— 位置控制飞去目标 + 速度P伺服微调 + 编排下游链。

由 SearchState 在 investigate 验证通过后 push 进来。

内部阶段:
  transit          → 位置控制飞向目标（视觉跟踪持续更新）
  servo            → 速度 P 控制精调
  [下游链]         → descend → fine_align → drop
  done             → SearchState resume

参数:
  bottle_index: 1 或 2，由 SearchState 传入
"""

import math

from mavsdk.offboard import PositionNedYaw, VelocityNedYaw

from .base_state import BaseState, ExecutionResult
from .align_precise import AlignPreciseState
from .drop import DropState
from config import (
    DROP_ALIGN_ALTITUDE_M,
    ALIGN_THRESHOLD_M,
    ARRIVAL_THRESHOLD_M,
    BUCKET_HEIGHT_M,
    VISUAL_SERVO_KP,
)
from vision.camera import capture_frame_async
from vision.yolo_detector import YOLODetector
from vision.circle_detector import CircleDetector


class AlignState(BaseState):
    """粗对准 + 编排 descend→fine_align→drop 链。"""

    def __init__(self, bottle_index: int, timeout_s: float = 90):
        super().__init__("Align", timeout_s)
        self.bottle_index = bottle_index

        # ---- 内部阶段 ----
        self._phase: str = "transit"  # transit → servo → [chain]

        # ---- 下游链 ----
        self._chain_step: str = ""    # "descend" → "fine_align" → "drop" → "done"

        # ---- 目标追踪 ----
        self._target = None
        self._enter_position = None
        self._enter_alt: float = 0.0

        # ---- 视觉 ----
        self.yolo_detector = YOLODetector()
        self.circle_detector = CircleDetector()

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def enter(self, interface):
        await super().enter(interface)

        # 记录进入位置
        pos = await interface.get_position_ned()
        self._enter_position = pos
        self._enter_alt = await interface.get_altitude()

        # 自行检测目标（search 不填 drop_targets 内容）
        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets
                           if d["cls"] == 1 and d["conf"] >= 0.3]

            if not bucket_dets:
                self.error = "视野中无桶目标"
                print("[粗对准] 错误: 无桶目标")
                return

            # 对每个桶做圆检测
            candidates = []
            for det in bucket_dets:
                bbox = (det["x1"], det["y1"], det["x2"], det["y2"])
                cr = self.circle_detector.detect(frame, bbox)
                if cr is not None:
                    d_m = self.circle_detector.compute_diameter(
                        cr.radius_px, alt)
                    if d_m > 0:
                        candidates.append({
                            "cx_px": cr.cx_px, "cy_px": cr.cy_px,
                            "diameter_m": d_m,
                        })

            if not candidates:
                self.error = "未检测到有效圆"
                print("[粗对准] 错误: 无有效圆")
                return

            # 选匹配 15cm/20cm 的最近目标
            best = None
            for c in candidates:
                d_cm = c["diameter_m"] * 100
                if abs(d_cm - 15) < 2 or abs(d_cm - 20) < 2:
                    best = c; break

            if best is None:
                self.error = "未检测到 15cm/20cm 目标"
                print(f"[粗对准] 错误: {self.error}")
                return

            # NED 偏移
            fx = self.circle_detector.fx; fy = self.circle_detector.fy
            cx = self.circle_detector.cx; cy = self.circle_detector.cy
            z_c = alt - BUCKET_HEIGHT_M
            e = (best["cx_px"] - cx) * z_c / fx
            n = -(best["cy_px"] - cy) * z_c / fy

            self._target = {"ned_offset": (n, e)}
            print(f"[粗对准] 目标 NED 偏移: N({n:.3f}) E({e:.3f}) 米")

        except Exception as e:
            self.error = f"视觉检测失败: {e}"
            print(f"[粗对准] 错误: {self.error}")
            return

        # 初始位置 setpoint
        n, e = self._target["ned_offset"]
        dn, de = self._rotate_offset(n, e, interface.FIELD_YAW_DEG)
        sp = PositionNedYaw(
            self._enter_position.north_m + dn,
            self._enter_position.east_m + de,
            -self._enter_alt, interface.FIELD_YAW_DEG)
        interface.update_setpoint(sp)

        self._phase = "transit"
        print("[粗对准] 进入 transit (位置控制)")

    async def execute(self, interface):
        if self.error:
            print(f"[粗对准] 错误退出: {self.error}")
            return ExecutionResult(done=True)

        if self.is_timed_out():
            self.error = "粗对准超时"
            print("[粗对准] 超时退出")
            return ExecutionResult(done=True)

        # ---- 下游链编排 ----
        if self._chain_step == "descend":
            self._chain_step = "fine_align"
            print(f"[粗对准] 调度: 下降至 {DROP_ALIGN_ALTITUDE_M}m")
            return ExecutionResult(interrupt=DescendState(self.bottle_index))

        elif self._chain_step == "fine_align":
            self._chain_step = "drop"
            print(f"[粗对准] 调度: 精细对准 (瓶子 {self.bottle_index})")
            return ExecutionResult(interrupt=AlignPreciseState(
                bottle_index=self.bottle_index, skip_descend=True))

        elif self._chain_step == "drop":
            self._chain_step = "done"
            print(f"[粗对准] 调度: 投放 (瓶子 {self.bottle_index})")
            return ExecutionResult(interrupt=DropState(
                bottle_index=self.bottle_index))

        elif self._chain_step == "done":
            print("[粗对准] 链完成 → 退出")
            self.is_completed = True
            return ExecutionResult(done=True)

        # ---- 视觉跟踪（transit + servo 共用） ----
        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets
                           if d["cls"] == 1 and d["conf"] >= 0.3]

            if bucket_dets:
                fx = self.circle_detector.fx; fy = self.circle_detector.fy
                cx = self.circle_detector.cx; cy = self.circle_detector.cy
                t_n, t_e = self._target["ned_offset"]

                best_match, best_dist = None, float('inf')
                for det in bucket_dets:
                    bbox = (det["x1"], det["y1"], det["x2"], det["y2"])
                    cr = self.circle_detector.detect(frame, bbox)
                    if cr is not None:
                        z_c = alt - BUCKET_HEIGHT_M
                        e = (cr.cx_px - cx) * z_c / fx
                        n = -(cr.cy_px - cy) * z_c / fy
                        d = (n - t_n)**2 + (e - t_e)**2
                        if d < best_dist:
                            best_dist = d
                            best_match = {"ned_offset": (n, e)}

                if best_match and best_dist < 1.0:
                    self._target["ned_offset"] = best_match["ned_offset"]

        except Exception as e:
            print(f"[粗对准] 视觉异常: {e}")

        # ---- 阶段分发 ----
        if self._phase == "transit":
            return await self._do_transit(interface)
        elif self._phase == "servo":
            return await self._do_servo(interface)

        return ExecutionResult()

    async def _do_transit(self, interface):
        if self._target is None:
            return ExecutionResult()

        n, e = self._target["ned_offset"]
        if abs(n) < ARRIVAL_THRESHOLD_M and abs(e) < ARRIVAL_THRESHOLD_M:
            self._phase = "servo"
            print(f"[粗对准] 到达目标附近 → 速度P伺服")
            return ExecutionResult()

        dn, de = self._rotate_offset(n, e, interface.FIELD_YAW_DEG)
        sp = PositionNedYaw(
            self._enter_position.north_m + dn,
            self._enter_position.east_m + de,
            -self._enter_alt, interface.FIELD_YAW_DEG)
        interface.update_setpoint(sp)
        return ExecutionResult()

    async def _do_servo(self, interface):
        if self._target is None:
            return ExecutionResult()

        n, e = self._target["ned_offset"]
        if abs(n) < ALIGN_THRESHOLD_M and abs(e) < ALIGN_THRESHOLD_M:
            interface.clear_velocity()
            self._chain_step = "descend"
            self._phase = ""
            print("[粗对准] 对准成功 → 启动下游链")
            return ExecutionResult()

        vn = VISUAL_SERVO_KP * n
        ve = VISUAL_SERVO_KP * e
        vn, ve = self._rotate_offset(vn, ve, interface.FIELD_YAW_DEG)
        vel = VelocityNedYaw(vn, ve, 0.0, interface.FIELD_YAW_DEG)
        interface.update_velocity_setpoint(vel)
        return ExecutionResult()

    async def exit(self, interface):
        interface.clear_velocity()
        await super().exit(interface)

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------

    @staticmethod
    def _rotate_offset(n, e, yaw_deg):
        theta = math.radians(yaw_deg)
        dn = n * math.cos(theta) - e * math.sin(theta)
        de = n * math.sin(theta) + e * math.cos(theta)
        return dn, de


class DescendState(BaseState):
    """下降到投放高度。"""

    def __init__(self, bottle_index: int, timeout_s: float = 30):
        super().__init__("Descend", timeout_s)
        self.bottle_index = bottle_index

    async def execute(self, interface):
        alt = await interface.get_altitude()
        if alt <= DROP_ALIGN_ALTITUDE_M:
            print(f"[下降] 到达投放高度: {alt:.2f}m")
            self.is_completed = True
            return ExecutionResult(done=True)

        if self.is_timed_out():
            self.error = "下降超时"
            return ExecutionResult(done=True)

        # 保持当前水平位置，只降高度
        pos = await interface.get_position_ned()
        sp = PositionNedYaw(
            pos.north_m, pos.east_m,
            -DROP_ALIGN_ALTITUDE_M, interface.FIELD_YAW_DEG)
        interface.update_setpoint(sp)
        return ExecutionResult()

    async def exit(self, interface):
        interface.clear_velocity()
        await super().exit(interface)
