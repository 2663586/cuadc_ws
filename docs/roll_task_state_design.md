# ============================================================
# roll_task.py —— 独立视觉决策的投放任务状态 (保姆级伪代码)
# ============================================================
# goal 三态语义:
#   shared["goal"] = [None, None]   ← SearchState 初始化
#   None  → 未被搜索状态发现
#   0     → 已发现，等待投桶
#   1     → 已投完
#
# 栈式调用关系:
#   SearchState 发现目标 → goal[?] None改为0 → push RollTaskState()
#   RollTaskState 读 goal 决定投哪个 (15优先):
#     goal[0]==0 → 投15cm → 完成后 goal[0]=1
#     goal[1]==0 → 投20cm → 完成后 goal[1]=1
#   RollTask done → pop, SearchState resume
#   goal==[1,1] → SearchState interrupt=ReconState (飞往侦查区)
# ============================================================

# ============================================================
# 1. 导入依赖
# ============================================================
import asyncio
import math

from mavsdk.offboard import VelocityNedYaw

from .base_state import BaseState, ExecutionResult
from .hover import HoverState
from config import (
    DROP_ALIGN_ALTITUDE_M,      # 投放对准高度 (2.5米)
    ALIGN_THRESHOLD_M,          # 对准精度阈值 (0.05米)
    ARRIVAL_THRESHOLD_M,        # 粗到位到达阈值 (0.5米)
    SEARCH_TIMEOUT_S,           # 视觉丢失超时 (3秒)
    VISUAL_SERVO_KP,            # 视觉伺服 P 增益
    DROP_SERVO_CHANNEL,         # ★ 舵机通道号 (从 config.py 读取)
    USE_SERVO,                  # ★ 是否启用舵机 (True/False, 便于调试)
)
from logger_manager import get_logger
from vision.camera import capture_frame_async
from vision.yolo_detector import YOLODetector      # YOLO 目标检测
from vision.circle_detector import CircleDetector  # 边缘检测 + 直径计算


# ============================================================
# 2. 状态类定义
# ============================================================
class RollTaskState(BaseState):
    """
    独立视觉决策的投放任务状态。

    由 SearchState 在检测到目标后通过栈式抢占 push 进来。
    enter() 时读取 shared["goal"] 决定本次投哪个桶 (15cm 优先)。

    ★ 不靠构造参数 —— 自己读 goal 做判断。
    """

    # ------------------------------------------------------------
    # 2.1 初始化
    # ------------------------------------------------------------
    def __init__(self, timeout_s: float = 90):
        """初始化状态，内部变量。target_type 在 enter() 中根据 goal 决定。"""
        super().__init__("ROLL_TASK", timeout_s)

        # ---- 本次任务目标 (在 enter() 中根据 goal 决定) ----
        self.target_type: str = ""    # "15" or "20"

        # ---- 内部阶段控制 ----
        self.phase: str = "transit"   # transit → align → descend → drop → return_home

        # ---- 目标追踪数据 ----
        self._target = None
        self._search_start: float = 0.0
        self._enter_position = None
        self._enter_alt: float = 0.0     # 进入时的高度 (PositionNedYaw 无 .up 字段)

        # ---- 任务状态标志 ----
        self._drop_done: bool = False
        self._descend_start_time: float = 0.0

        # ---- 视觉检测器 ----
        self.yolo_detector = YOLODetector()
        self.circle_detector = CircleDetector()

    # ------------------------------------------------------------
    # 2.2 进入状态 (enter)
    # ------------------------------------------------------------
    async def enter(self, interface):
        """
        状态进入时调用一次。
        1. 读 shared["goal"] 决定本次投哪个桶 (15cm 优先)
        2. 自行检测目标，选择匹配的目标
        3. 计算 NED 偏移，设定初始位置
        """
        await super().enter(interface)

        # ---- 步骤 1: 记录进入位置 (POSITION_LEFT) ----
        pos = await interface.get_position_ned()
        self._enter_position = pos
        self._enter_alt = await interface.get_altitude()
        self._search_start = self.elapsed()
        print("[ROLL_TASK] 已记录进入位置 (POSITION_LEFT)")

        # ---- 步骤 2: 读取 goal 标志，确定本次任务目标 (15cm 优先) ----
        goal = interface.shared.get("goal")
        if goal is None:
            self.error = "goal 标志未初始化"
            print("[ROLL_TASK] 错误: goal 标志未初始化")
            return

        match goal:
            case [0, _]:
                self.target_type = "15"
                print("[ROLL_TASK] 根据 goal 标志，本次任务: 投放 15cm 桶")
            case [_, 0]:
                self.target_type = "20"
                print("[ROLL_TASK] 根据 goal 标志，本次任务: 投放 20cm 桶")
            case _:
                # goal[0] 和 goal[1] 都不是 0: 可能都是 None(没找到) 或都是 1(已投完)
                # 正常情况下不会进入这里——SearchState 只在 goal=0 时 push
                self.error = "无可投放目标 (goal 中无 0)"
                print(f"[ROLL_TASK] 错误: 无可投放目标, goal={goal}")
                return

        # ---- 步骤 3: 自行检测目标 (YOLO + CircleDetector) ----
        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            # 3a. YOLO 检测桶的边界框
            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets if d["cls"] == 1 and d["conf"] >= 0.3]

            if not bucket_dets:
                self.error = "视野中无任何桶目标"
                print("[ROLL_TASK] 错误: 无桶目标")
                return

            # 3b. 对每个桶进行边缘检测和圆检测，计算真实直径
            detected_targets = []
            for det in bucket_dets:
                bbox = (det["x1"], det["y1"], det["x2"], det["y2"])
                circle_result = self.circle_detector.detect(frame, bbox)
                if circle_result is not None:
                    diameter_m = self.circle_detector.compute_diameter(
                        circle_result.radius_px, alt
                    )
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
            print(f"[ROLL_TASK] 错误: 未检测到 {self.target_type}cm 目标")
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

        # ---- 步骤 6: 设定初始位置 setpoint ----
        sp = interface.field_to_ned(
            self._enter_position.north_m + offset_north,
            self._enter_position.east_m + offset_east,
            self._enter_alt,
        )
        interface.update_setpoint(sp)

        self.phase = "transit"
        print("[ROLL_TASK] 进入粗到位阶段 (位置控制 → PX4 原生飞往目标)")

    # ------------------------------------------------------------
    # 2.3 主循环 (execute)
    # ------------------------------------------------------------
    async def execute(self, interface):
        """
        状态主循环，每帧调用。
        返回 ExecutionResult:
          - done=True          → 弹出自身，栈下层 SearchState 自动 resume
          - interrupt=HoverState → 视觉丢失，切换悬停
          - ExecutionResult()  → 继续执行
        """
        # ---- enter() 报错时立即退出，避免空转 ----
        if self.error:
            print(f"[ROLL_TASK] enter() 阶段失败: {self.error}，退出")
            return ExecutionResult(done=True)

        # ---- 总超时检查 ----
        if self.is_timed_out():
            self.error = "ROLL_TASK 总超时"
            print("[ROLL_TASK] 总超时退出")
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
        elif self.phase == "drop":
            return await self._do_drop(interface)
        elif self.phase == "return_home":
            return await self._do_return_home(interface)

        return ExecutionResult()

    # ------------------------------------------------------------
    # 2.4 视觉丢失检测
    # ------------------------------------------------------------
    async def _check_visual_loss(self, interface):
        """
        检查目标是否丢失。
        - 丢失超时 → 返回 ExecutionResult(interrupt=HoverState())
        - 正常追踪 → 返回 None，继续当前阶段
        """
        if self._target is None:
            return None

        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets if d["cls"] == 1 and d["conf"] >= 0.3]

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

            # 寻找与当前目标最匹配的检测
            best_match = None
            best_distance = float('inf')
            t_north, t_east = self._target["ned_offset"]

            for det in bucket_dets:
                bbox = (det["x1"], det["y1"], det["x2"], det["y2"])
                circle_result = self.circle_detector.detect(frame, bbox)
                if circle_result is not None:
                    e  = (circle_result.cx_px - cx) * alt / fx
                    n  = -(circle_result.cy_px - cy) * alt / fy
                    dn = n - t_north
                    de = e - t_east
                    dist = dn*dn + de*de
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

            # 没有匹配到目标
            if self.elapsed() - self._search_start > SEARCH_TIMEOUT_S:
                self.error = "视觉丢失超时 (匹配失败)"
                print("[视觉丢失] 匹配失败且超时，切换 HOVER")
                return ExecutionResult(interrupt=HoverState())
            return None

        except Exception as e:
            print(f"[视觉丢失] 检测异常: {e}")
            return None

    # ------------------------------------------------------------
    # 2.5 粗到位阶段 (位置控制 → PX4 原生飞往目标)
    # ------------------------------------------------------------
    async def _do_transit(self, interface):
        """
        位置控制粗到位: 每帧更新位置 setpoint，持续飞向目标。
        距离 < ARRIVAL_THRESHOLD_M 后切换到速度 P 控制精调。

        ★ 方案 A: 每帧视觉检测 → 更新 setpoint，PX4 内部 200Hz+ 飞控执行。
        """
        if self._target is None:
            return ExecutionResult()

        # ---- 视觉检测：持续更新目标位置 (与 align 逻辑一致，只是控制模式不同) ----
        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets if d["cls"] == 1 and d["conf"] >= 0.3]

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
                        dist = dn*dn + de*de
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
        if (abs(offset_north) < ARRIVAL_THRESHOLD_M
                and abs(offset_east) < ARRIVAL_THRESHOLD_M):
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

    # ------------------------------------------------------------
    # 2.6 对准阶段 (视觉伺服 — 速度 P 控制)
    # ------------------------------------------------------------
    async def _do_align(self, interface):
        """视觉伺服对准: 速度 P 控制微调最后几十厘米。"""
        if self._target is None:
            return ExecutionResult()

        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            yolo_dets = self.yolo_detector.detect(frame)
            bucket_dets = [d for d in yolo_dets if d["cls"] == 1 and d["conf"] >= 0.3]

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
                        dist = dn*dn + de*de
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
        if (abs(offset_north) < ALIGN_THRESHOLD_M
                and abs(offset_east) < ALIGN_THRESHOLD_M):
            # ★ 切回位置模式，让 descend 阶段用位置控制下降
            interface.clear_velocity()
            self.phase = "descend"
            self._descend_start_time = self.elapsed()
            print(f"[对准] 对准成功，开始下降")
            return ExecutionResult()

        # ---- 发送速度指令 (P 控制, NED 帧) ----
        vn = VISUAL_SERVO_KP * offset_north
        ve = VISUAL_SERVO_KP * offset_east
        vel = VelocityNedYaw(vn, ve, 0.0, interface.FIELD_YAW_DEG)
        interface.update_velocity_setpoint(vel)

        return ExecutionResult()

    # ------------------------------------------------------------
    # 2.7 下降阶段
    # ------------------------------------------------------------
    async def _do_descend(self, interface):
        """垂直下降到投放高度，保持水平位置。"""
        alt = await interface.get_altitude()
        target_alt = DROP_ALIGN_ALTITUDE_M

        if alt <= target_alt:
            self.phase = "drop"
            print(f"[下降] 到达投放高度: {alt:.2f}m")
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
                DROP_ALIGN_ALTITUDE_M,  # ← 直接设为目标高度
            )
            interface.update_setpoint(sp)

        return ExecutionResult()

    # ------------------------------------------------------------
    # 2.8 投桶阶段 ★ 更新 goal 标志
    # ------------------------------------------------------------
    async def _do_drop(self, interface):
        """
        执行投桶动作，完成后更新 shared["goal"]，然后进入返回阶段。

        ★ 只投自己的 target_type，不判断是否有剩余目标。
        ★ shared["goal"] 更新后，SearchState resume 时会自动看到。
        """
        if self._drop_done:
            self.phase = "return_home"
            return ExecutionResult()

        # ---- 1. 执行投桶动作 (舵机控制) ----
        print("[投桶] 执行投桶...")
        if USE_SERVO:
            await interface.set_actuator(DROP_SERVO_CHANNEL, 1.0)
            await asyncio.sleep(0.5)
            await interface.set_actuator(DROP_SERVO_CHANNEL, -1.0)
        else:
            print("[投桶] 舵机已禁用 (USE_SERVO=False)，模拟投桶完成")

        self._drop_done = True
        print("[投桶] 投桶完成")

        # ---- 2. ★ 更新共享 goal 标志: 0 → 1 (投完了) ----
        goal = interface.shared.get("goal")
        if goal is not None:
            if self.target_type == "15":
                goal[0] = 1
                print("[ROLL_TASK] goal[0]: 0→1, 15cm 已投放")
            else:  # "20"
                goal[1] = 1
                print("[ROLL_TASK] goal[1]: 0→1, 20cm 已投放")
        else:
            print("[ROLL_TASK] 警告: shared['goal'] 未初始化，跳过更新")

        # ---- 3. 进入返回阶段 ----
        self.phase = "return_home"
        return ExecutionResult()

    # ------------------------------------------------------------
    # 2.9 返回阶段
    # ------------------------------------------------------------
    async def _do_return_home(self, interface):
        """
        返回进入 ROLL_TASK 时的位置 (POSITION_LEFT)。

        ★ done=True 弹出自身，栈下层 SearchState 自动 resume。
        ★ SearchState 会看到更新后的 shared["goal"]。
        """
        current_pos = await interface.get_position_ned()

        dx = current_pos.north_m - self._enter_position.north_m
        dy = current_pos.east_m - self._enter_position.east_m
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.3:
            self.is_completed = True
            print("[返回] 已回到 POSITION_LEFT，弹出自身 → SearchState resume")
            return ExecutionResult(done=True)

        sp = interface.field_to_ned(
            self._enter_position.north_m,
            self._enter_position.east_m,
            self._enter_alt,
        )
        interface.update_setpoint(sp)
        return ExecutionResult()
