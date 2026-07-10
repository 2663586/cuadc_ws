"""
投放任务状态 —— 粗对准 + 下降 + 调度 HSK 精细组件。

由 SearchState 在检测到目标后通过栈式抢占 push 进来。
enter() 时读取 shared["goal"] 决定本次投哪个桶 (15cm 优先)。

内部阶段:
  transit          → 粗到位（位置控制）
  align            → 粗对准（速度 P 伺服，判据 0.1m）
  descend          → 下降到 2.5m（之后交接给 HSK）
  [HSK 调度链]     → 精细对准 → 投放 → 分流返航（由 _hsk_step 编排）
  done             → 弹出自身，SearchState resume 接管后续
                     (goal=[1,1] 时 SearchState 自动切入 ReconState)

goal 三态:
    None → 未被搜索状态发现
    0    → 已发现，等待投桶
    1    → 已投完
"""

import math

from mavsdk.offboard import VelocityNedYaw

from .base_state import BaseState, ExecutionResult
from .hover import HoverState
from .align_precise import AlignPreciseState
from .drop import DropState
from .post_drop_nav import PostDropNavState
from config import (
    DROP_ALIGN_ALTITUDE_M,
    ARRIVAL_THRESHOLD_M,
    SEARCH_TIMEOUT_S,
    VISUAL_SERVO_KP,
)
from vision.camera import capture_frame_async
from vision.yolo_detector import YOLODetector
from vision.circle_detector import CircleDetector


class RollTaskState(BaseState):
    """粗对准 + 下降 + HSK 组件编排。"""

    # 粗对准参数
    COARSE_ALIGN_THRESHOLD_M = 0.1  # 粗对准完成判据 (NED偏移, 米)

    def __init__(self, timeout_s: float = 90):
        super().__init__("ROLL_TASK", timeout_s)

        # ---- 本次任务目标 (在 enter() 中根据 goal 决定) ----
        self.target_type: str = ""  # "15" or "20"

        # ---- 内部阶段控制 ----
        self.phase: str = "transit"  # transit → align → descend → [HSK调度] → judge

        # ---- HSK 调度链 ----
        self._hsk_step: str = ""     # "fine_align" → "drop" → "navigate" → "done"
        self._active_bottle: int = 0  # 当前处理的瓶子编号 (1 or 2)
        self._going_to_recon: bool = False  # Drop后判定: True=飞侦察区, False=回中断点

        # ---- 目标追踪数据 ----
        self._target = None
        self._search_start: float = 0.0
        self._enter_position = None
        self._enter_alt: float = 0.0

        # ---- 任务状态标志 ----
        self._descend_start_time: float = 0.0

        # ---- 视觉检测器 ----
        self.yolo_detector = YOLODetector()
        self.circle_detector = CircleDetector()

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def enter(self, interface):
        """进入状态：读 goal 决定投哪个 → 检测目标 → 设初始位置。"""
        await super().enter(interface)

        # ---- 步骤 1: 记录进入位置 ----
        pos = await interface.get_position_ned()
        self._enter_position = pos
        self._enter_alt = await interface.get_altitude()
        self._search_start = self.elapsed()
        print("[ROLL_TASK] 已记录进入位置")

        # ---- 步骤 2: 读取 goal 标志，确定本次任务目标 (15cm 优先) ----
        goal = interface.shared.get("goal")
        if goal is None:
            self.error = "goal 标志未初始化"
            print("[ROLL_TASK] 错误: goal 标志未初始化")
            return

        match goal:
            case [0, _] if goal[0] == 0:
                self.target_type = "15"
                self._active_bottle = 1
                print("[ROLL_TASK] 根据 goal 标志，本次任务: 投放 15cm 桶")
            case [_, 0] if goal[1] == 0:
                self.target_type = "20"
                self._active_bottle = 2
                print("[ROLL_TASK] 根据 goal 标志，本次任务: 投放 20cm 桶")
            case _:
                self.error = "无可投放目标 (goal 中无 0)"
                print(f"[ROLL_TASK] 错误: 无可投放目标, goal={goal}")
                return

        # ---- 步骤 3: 自行检测目标 (YOLO + CircleDetector) ----
        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets
                           if d["cls"] == 1 and d["conf"] >= 0.3]

            if not bucket_dets:
                self.error = "视野中无任何桶目标"
                print("[ROLL_TASK] 错误: 无桶目标")
                return

            detected_targets = []
            for det in bucket_dets:
                bbox = (det["x1"], det["y1"], det["x2"], det["y2"])
                circle_result = self.circle_detector.detect(frame, bbox)
                if circle_result is not None:
                    diameter_m = self.circle_detector.compute_diameter(
                        circle_result.radius_px, alt)
                    if diameter_m > 0:
                        detected_targets.append({
                            "bbox": bbox,
                            "cx_px": circle_result.cx_px,
                            "cy_px": circle_result.cy_px,
                            "diameter_m": diameter_m,
                        })

            if not detected_targets:
                self.error = "未检测到有效圆 (边缘检测失败)"
                print("[ROLL_TASK] 错误: 无有效圆")
                return

        except Exception as e:
            self.error = f"视觉检测失败: {e}"
            print(f"[ROLL_TASK] 错误: {self.error}")
            return

        # ---- 步骤 4: 从检测结果中选择匹配 self.target_type 的目标 ----
        target_match = None
        for t in detected_targets:
            diameter_cm = t["diameter_m"] * 100
            if self.target_type == "15" and abs(diameter_cm - 15) < 2:
                target_match = t
                break
            elif self.target_type == "20" and abs(diameter_cm - 20) < 2:
                target_match = t
                break

        if target_match is None:
            self.error = f"未检测到 {self.target_type}cm 目标"
            print(f"[ROLL_TASK] 错误: {self.error}")
            return

        self._target = target_match
        print(f"[ROLL_TASK] 选择 {self.target_type}cm 目标 "
              f"(直径 {target_match['diameter_m']*100:.1f}cm)")

        # ---- 步骤 5: 像素坐标 → NED 偏移 (与 pixel_to_ned_offset 一致) ----
        fx = self.circle_detector.fx
        fy = self.circle_detector.fy
        cx = self.circle_detector.cx
        cy = self.circle_detector.cy

        # 图像右=场地东, 图像下=场地南=-北
        offset_east  = (self._target["cx_px"] - cx) * alt / fx
        offset_north = -(self._target["cy_px"] - cy) * alt / fy

        self._target["ned_offset"] = (offset_north, offset_east)
        print(f"[ROLL_TASK] 目标 NED 偏移: "
              f"N({offset_north:.3f}) E({offset_east:.3f}) 米")

        # ---- 步骤 6: 桥接数据到 shared，供 HSK 组件读取 ----
        from .search import CylinderTarget
        bridge_target = CylinderTarget(
            ned_offset=(offset_north, offset_east),
            diameter_m=target_match["diameter_m"],
            conf=0.0,  # RollTaskState 不追踪置信度
            circle_cx_px=target_match["cx_px"],
            circle_cy_px=target_match["cy_px"],
        )
        if "drop_targets" not in interface.shared:
            interface.shared["drop_targets"] = [None, None]
        interface.shared["drop_targets"][self._active_bottle - 1] = bridge_target
        print(f"[ROLL_TASK] 目标已桥接到 shared['drop_targets'][{self._active_bottle - 1}]")

        # ---- 步骤 7: 设定初始位置 setpoint ----
        sp = interface.field_to_ned(
            self._enter_position.north_m + offset_north,
            self._enter_position.east_m + offset_east,
            self._enter_alt,
        )
        interface.update_setpoint(sp)

        self.phase = "transit"
        print("[ROLL_TASK] 进入粗到位阶段 (位置控制)")

    async def execute(self, interface):
        """主循环，每帧调用。"""
        # ---- enter() 报错时立即退出 ----
        if self.error:
            print(f"[ROLL_TASK] enter() 阶段失败: {self.error}，退出")
            return ExecutionResult(done=True)

        # ---- 总超时 ----
        if self.is_timed_out():
            self.error = "ROLL_TASK 总超时"
            print("[ROLL_TASK] 总超时退出")
            return ExecutionResult(done=True)

        # ---- HSK 组件调度链 (descend 完成后触发) ----
        if self._hsk_step == "fine_align":
            self._hsk_step = "drop"
            print(f"[ROLL_TASK] 调度 HSK: 精细对准 (瓶子 {self._active_bottle})")
            return ExecutionResult(interrupt=AlignPreciseState(
                bottle_index=self._active_bottle, skip_descend=True))

        elif self._hsk_step == "drop":
            self._hsk_step = "navigate"
            print(f"[ROLL_TASK] 调度 HSK: 投放 (瓶子 {self._active_bottle})")
            return ExecutionResult(interrupt=DropState(
                bottle_index=self._active_bottle))

        elif self._hsk_step == "navigate":
            # ★ 投放完成，goal 已更新，判断去向
            goal = interface.shared.get("goal", [None, None])
            self._hsk_step = "done"
            if goal == [1, 1]:
                print("[ROLL_TASK] 两轮投放均完成 → 上升后前往侦察区")
                self._going_to_recon = True
                return ExecutionResult(interrupt=PostDropNavState(
                    destination="recon"))
            else:
                print(f"[ROLL_TASK] goal={goal} → 上升后返回中断点继续搜索")
                self._going_to_recon = False
                return ExecutionResult(interrupt=PostDropNavState(
                    destination="return"))

        elif self._hsk_step == "done":
            # 返航完成
            if self._going_to_recon:
                print("[ROLL_TASK] 已到达侦察区上空 → 退出，"
                      "SearchState 接管侦察")
            else:
                print("[ROLL_TASK] 已返回中断点 → 回到 SearchState 继续搜索")
            self.is_completed = True
            return ExecutionResult(done=True)

        # ---- 视觉丢失检查 (transit/align/descend 阶段) ----
        if self.phase in ("transit", "align", "descend"):
            result = await self._check_visual_loss(interface)
            if result is not None:
                return result

        # ---- 阶段分发 ----
        if self.phase == "transit":
            return await self._do_transit(interface)
        elif self.phase == "align":
            return await self._do_align(interface)
        elif self.phase == "descend":
            return await self._do_descend(interface)

        return ExecutionResult()

    # ------------------------------------------------------------------
    # 视觉丢失检测
    # ------------------------------------------------------------------

    async def _check_visual_loss(self, interface):
        """检查目标是否丢失。丢失超时 → interrupt=HoverState()。"""
        if self._target is None:
            return None

        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets
                           if d["cls"] == 1 and d["conf"] >= 0.3]

            if not bucket_dets:
                if self.elapsed() - self._search_start > SEARCH_TIMEOUT_S:
                    self.error = "视觉丢失超时 (无目标)"
                    print("[视觉丢失] 无目标且超时，切换 HOVER")
                    return ExecutionResult(interrupt=HoverState())
                return None

            fx = self.circle_detector.fx
            fy = self.circle_detector.fy
            cx = self.circle_detector.cx
            cy = self.circle_detector.cy
            t_north, t_east = self._target["ned_offset"]

            best_match = None
            best_distance = float('inf')

            for det in bucket_dets:
                bbox = (det["x1"], det["y1"], det["x2"], det["y2"])
                circle_result = self.circle_detector.detect(frame, bbox)
                if circle_result is not None:
                    e  = (circle_result.cx_px - cx) * alt / fx
                    n  = -(circle_result.cy_px - cy) * alt / fy
                    dn = n - t_north
                    de = e - t_east
                    dist = dn * dn + de * de
                    if dist < best_distance:
                        best_distance = dist
                        best_match = {
                            "ned_offset": (n, e),
                            "cx_px": circle_result.cx_px,
                            "cy_px": circle_result.cy_px,
                        }

            if best_match and best_distance < 1.0:
                self._target["ned_offset"] = best_match["ned_offset"]
                self._target["cx_px"] = best_match["cx_px"]
                self._target["cy_px"] = best_match["cy_px"]
                self._search_start = self.elapsed()
                return None

            if self.elapsed() - self._search_start > SEARCH_TIMEOUT_S:
                self.error = "视觉丢失超时 (匹配失败)"
                print("[视觉丢失] 匹配失败且超时，切换 HOVER")
                return ExecutionResult(interrupt=HoverState())
            return None

        except Exception as e:
            print(f"[视觉丢失] 检测异常: {e}")
            return None

    # ------------------------------------------------------------------
    # 粗到位阶段 (位置控制 → PX4 原生飞往目标)
    # ------------------------------------------------------------------

    async def _do_transit(self, interface):
        """
        位置控制粗到位: 每帧更新位置 setpoint，持续飞向目标。
        距离 < ARRIVAL_THRESHOLD_M 后切换到速度 P 控制精调。
        """
        if self._target is None:
            return ExecutionResult()

        # ---- 视觉检测：持续更新目标位置 ----
        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets
                           if d["cls"] == 1 and d["conf"] >= 0.3]

            if bucket_dets:
                fx = self.circle_detector.fx
                fy = self.circle_detector.fy
                cx = self.circle_detector.cx
                cy = self.circle_detector.cy
                t_north, t_east = self._target["ned_offset"]

                best_match = None
                best_distance = float('inf')

                for det in bucket_dets:
                    bbox = (det["x1"], det["y1"], det["x2"], det["y2"])
                    circle_result = self.circle_detector.detect(frame, bbox)
                    if circle_result is not None:
                        e  = (circle_result.cx_px - cx) * alt / fx
                        n  = -(circle_result.cy_px - cy) * alt / fy
                        dn = n - t_north
                        de = e - t_east
                        dist = dn * dn + de * de
                        if dist < best_distance:
                            best_distance = dist
                            best_match = {
                                "ned_offset": (n, e),
                                "cx_px": circle_result.cx_px,
                                "cy_px": circle_result.cy_px,
                            }

                if best_match and best_distance < 1.0:
                    self._target["ned_offset"] = best_match["ned_offset"]
                    self._target["cx_px"] = best_match["cx_px"]
                    self._target["cy_px"] = best_match["cy_px"]
                    self._search_start = self.elapsed()

        except Exception as e:
            print(f"[粗到位] 视觉检测异常: {e}")
            return ExecutionResult()

        # ---- 检查是否到达目标附近 → 切速度精调 ----
        offset_north, offset_east = self._target["ned_offset"]
        if abs(offset_north) < ARRIVAL_THRESHOLD_M and abs(offset_east) < ARRIVAL_THRESHOLD_M:
            self.phase = "align"
            print(f"[粗到位] 到达目标附近 "
                  f"(N={offset_north:.2f} E={offset_east:.2f})，切换速度 P 控制")
            return ExecutionResult()

        # ---- 更新位置 setpoint: 持续飞向目标 ----
        sp = interface.field_to_ned(
            self._enter_position.north_m + offset_north,
            self._enter_position.east_m + offset_east,
            self._enter_alt,
        )
        interface.update_setpoint(sp)

        return ExecutionResult()

    # ------------------------------------------------------------------
    # 对准阶段 (视觉伺服 — 速度 P 控制)
    # ------------------------------------------------------------------

    async def _do_align(self, interface):
        """视觉伺服对准: 速度 P 控制，根据目标偏移发送速度指令。"""
        if self._target is None:
            return ExecutionResult()

        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets
                           if d["cls"] == 1 and d["conf"] >= 0.3]

            if bucket_dets:
                fx = self.circle_detector.fx
                fy = self.circle_detector.fy
                cx = self.circle_detector.cx
                cy = self.circle_detector.cy
                t_north, t_east = self._target["ned_offset"]

                best_match = None
                best_distance = float('inf')

                for det in bucket_dets:
                    bbox = (det["x1"], det["y1"], det["x2"], det["y2"])
                    circle_result = self.circle_detector.detect(frame, bbox)
                    if circle_result is not None:
                        e  = (circle_result.cx_px - cx) * alt / fx
                        n  = -(circle_result.cy_px - cy) * alt / fy
                        dn = n - t_north
                        de = e - t_east
                        dist = dn * dn + de * de
                        if dist < best_distance:
                            best_distance = dist
                            best_match = {
                                "ned_offset": (n, e),
                                "cx_px": circle_result.cx_px,
                                "cy_px": circle_result.cy_px,
                            }

                if best_match and best_distance < 1.0:
                    self._target["ned_offset"] = best_match["ned_offset"]
                    self._target["cx_px"] = best_match["cx_px"]
                    self._target["cy_px"] = best_match["cy_px"]
                    self._search_start = self.elapsed()

        except Exception as e:
            print(f"[对准] 视觉检测异常: {e}")
            return ExecutionResult()

        # ---- 检查对准精度 ----
        offset_north, offset_east = self._target["ned_offset"]
        if abs(offset_north) < self.COARSE_ALIGN_THRESHOLD_M and abs(offset_east) < self.COARSE_ALIGN_THRESHOLD_M:
            # ★ 切回位置模式，让 descend 阶段用位置控制下降
            interface.clear_velocity()
            self.phase = "descend"
            self._descend_start_time = self.elapsed()
            print("[对准] 对准成功，开始下降")
            return ExecutionResult()

        # ---- 发送速度指令 (P 控制) ----
        vn = VISUAL_SERVO_KP * offset_north
        ve = VISUAL_SERVO_KP * offset_east
        vel = VelocityNedYaw(vn, ve, 0.0, interface.FIELD_YAW_DEG)
        interface.update_velocity_setpoint(vel)

        return ExecutionResult()

    # ------------------------------------------------------------------
    # 下降阶段 → 交接给 HSK
    # ------------------------------------------------------------------

    async def _do_descend(self, interface):
        """垂直下降到投放高度，完成后启动 HSK 调度链。"""
        alt = await interface.get_altitude()

        if alt <= DROP_ALIGN_ALTITUDE_M:
            print(f"[下降] 到达投放高度: {alt:.2f}m → 交接给 HSK 精细对准")
            self._hsk_step = "fine_align"
            self.phase = ""  # 退出内部阶段，进入 HSK 调度
            return ExecutionResult()

        if self.elapsed() - self._descend_start_time > 15.0:
            self.error = "下降超时"
            print("[下降] 下降超时，退出")
            return ExecutionResult(done=True)

        if self._target is not None:
            offset_north, offset_east = self._target["ned_offset"]
            sp = interface.field_to_ned(
                self._enter_position.north_m + offset_north,
                self._enter_position.east_m + offset_east,
                DROP_ALIGN_ALTITUDE_M,
            )
            interface.update_setpoint(sp)

        return ExecutionResult()
