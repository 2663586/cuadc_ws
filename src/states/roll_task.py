"""
投放任务状态 —— 独立视觉决策的 对准 → 下降 → 投桶 → 返回。

由 SearchState 在检测到目标后通过栈式抢占 push 进来。
enter() 时读取 shared["goal"] 决定本次投哪个桶 (15cm 优先)。

goal 三态:
    None → 未被搜索状态发现
    0    → 已发现，等待投桶
    1    → 已投完

栈式流程:
    SearchState 发现目标 → goal[?]="None改为0" → push RollTaskState()
    RollTaskState 读 goal → goal[0]==0 投15 / goal[1]==0 投20
    投桶完成 goal[?]="0改1" → pop, SearchState resume
    goal==[1,1] → SearchState interrupt=ReconState
"""

import asyncio
import math

from mavsdk.offboard import PositionNedYaw, VelocityNedYaw

from .base_state import BaseState, ExecutionResult
from config import (
    DROP_ALIGN_ALTITUDE_M,
    ALIGN_THRESHOLD_M,
    ARRIVAL_THRESHOLD_M,
    BUCKET_HEIGHT_M,
    SEARCH_TIMEOUT_S,
    VISUAL_SERVO_KP,
    DROP_SERVO_CHANNEL,
    USE_SERVO,
)
from vision.camera import capture_frame_async
from vision.pipeline import VisionPipeline


class RollTaskState(BaseState):
    """独立视觉决策的投放任务状态。"""

    def __init__(self, timeout_s: float = 90, pid_n=None, pid_e=None):
        """
        pid_n, pid_e: 可选的独立 PID 控制器，分别控制 north/east 通道。
            需实现: pid.update(setpoint: float, measurement: float) -> float
            setpoint=0（对准目标），measurement=NED偏移(m)，返回速度指令(m/s)。
            不传则回退为内置 P 控制 (config.VISUAL_SERVO_KP)。
            ★ 两个通道独立，避免 I 项累积串扰。
        """
        super().__init__("ROLL_TASK", timeout_s)

        # ---- 本次任务目标 (在 enter() 中根据 goal 决定) ----
        self.target_type: str = ""  # "15" or "20"

        # ---- 内部阶段控制 ----
        self.phase: str = "transit"  # transit → align → descend → drop → return_home

        # ---- 目标追踪数据 ----
        self._target = None
        self._search_start: float = 0.0
        self._enter_position = None
        self._enter_alt: float = 0.0

        # ---- 任务状态标志 ----
        self._drop_done: bool = False
        self._descend_start_time: float = 0.0
        self._align_start_time: float = 0.0
        self._retrying: bool = False      # 视觉丢失后回 POSITION_LEFT 重试

        # ---- PID 控制器 (可选，各通道独立) ----
        self._pid_n = pid_n   # north 通道
        self._pid_e = pid_e   # east 通道

        # ---- 视觉流水线: YOLO → Canny边缘 → HoughCircles → 直径 ----
        self.pipeline = VisionPipeline(
            model_path="models/yolov11n_800_best_FP16.engine",
            yolo_conf=0.5,
            circle_conf_threshold=0.3,
        )

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
                self.error = "无可投放目标 (goal 中无 0)"
                print(f"[ROLL_TASK] 错误: 无可投放目标, goal={goal}")
                return

        # ---- 步骤 3: 自行检测目标 (VisionPipeline: YOLO→Canny→HoughCircles) ----
        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            results = self.pipeline.process_frame(frame, alt_rel_m=alt)
            detected_targets = [
                {
                    "bbox": (r["det"]["x1"], r["det"]["y1"],
                             r["det"]["x2"], r["det"]["y2"]),
                    "cx_px": r["circle"].cx_px,
                    "cy_px": r["circle"].cy_px,
                    "diameter_m": r["diameter_m"],
                }
                for r in results if r["edge_success"]
            ]

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
        fx = self.pipeline.circle.fx
        fy = self.pipeline.circle.fy
        cx = self.pipeline.circle.cx
        cy = self.pipeline.circle.cy

        # 图像右=场地东, 图像下=场地南=-北
        z_c = alt - BUCKET_HEIGHT_M
        offset_east  = (self._target["cx_px"] - cx) * z_c / fx
        offset_north = -(self._target["cy_px"] - cy) * z_c / fy

        self._target["ned_offset"] = (offset_north, offset_east)
        print(f"[ROLL_TASK] 目标 NED 偏移: "
              f"N({offset_north:.3f}) E({offset_east:.3f}) 米")

        # ---- 步骤 6: 设定初始位置 setpoint (offset 旋转到真北 NED) ----
        dn, de = self._rotate_offset(
            offset_north, offset_east, interface.FIELD_YAW_DEG)
        sp = PositionNedYaw(
            self._enter_position.north_m + dn,
            self._enter_position.east_m + de,
            -self._enter_alt,
            interface.FIELD_YAW_DEG,
        )
        interface.update_setpoint(sp)

        self.phase = "transit"
        print("[ROLL_TASK] 进入粗到位阶段 (位置控制)")

    async def execute(self, interface):
        """主循环，每帧调用。done=True 退回 SearchState。"""
        # ---- enter() 报错时立即退出，避免空转 ----
        if self.error:
            print(f"[ROLL_TASK] enter() 阶段失败: {self.error}，退出")
            return ExecutionResult(done=True)

        # ---- 总超时 ----
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

    # ------------------------------------------------------------------
    # 退出清理
    # ------------------------------------------------------------------

    async def exit(self, interface):
        """退出时清理：清速度、未投成则重置所有 goal=0 的项为 None。"""
        interface.clear_velocity()
        if not self._drop_done:
            goal = interface.shared.get("goal")
            if goal is not None:
                for i in (0, 1):
                    if goal[i] == 0:
                        goal[i] = None
                        print(f"[ROLL_TASK] 未投成，重置 goal[{i}]=None，允许重试")
        await super().exit(interface)

    # ------------------------------------------------------------------
    # 视觉丢失检测
    # ------------------------------------------------------------------

    async def _check_visual_loss(self, interface):
        """检查目标是否丢失。丢失超时 → 退回 POSITION_LEFT 重试。"""
        if self._target is None:
            return None

        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            match = self._match_target(interface, frame, alt)
            if match is None:
                if self.elapsed() - self._search_start > SEARCH_TIMEOUT_S:
                    self.error = "视觉丢失超时，退回 POSITION_LEFT 重试"
                    print("[视觉丢失] 目标丢失且超时，退回 POSITION_LEFT 重试")
                    interface.clear_velocity()
                    self.phase = "return_home"
                    self._retrying = True
                    return None
                return None

            self._target.update(match)
            self._search_start = self.elapsed()
            return None

        except Exception as e:
            print(f"[视觉丢失] 检测异常: {e}")
            return None

    # ------------------------------------------------------------------
    # 视觉匹配辅助
    # ------------------------------------------------------------------

    def _match_target(self, interface, frame, alt):
        """
        VisionPipeline 检测 + 目标匹配。匹配成功返回
        {"ned_offset": (n,e), "cx_px": int, "cy_px": int}，失败返回 None。
        """
        results = self.pipeline.process_frame(frame, alt_rel_m=alt)
        valid = [r for r in results if r["edge_success"]]
        if not valid:
            return None

        fx = self.pipeline.circle.fx
        fy = self.pipeline.circle.fy
        cx = self.pipeline.circle.cx
        cy = self.pipeline.circle.cy
        t_north, t_east = self._target["ned_offset"]

        best_match = None
        best_distance = float('inf')

        for r in valid:
            circle = r["circle"]
            z_c = alt - BUCKET_HEIGHT_M
            e = (circle.cx_px - cx) * z_c / fx
            n = -(circle.cy_px - cy) * z_c / fy
            dn = n - t_north
            de = e - t_east
            dist = dn * dn + de * de
            if dist < best_distance:
                best_distance = dist
                best_match = {
                    "ned_offset": (n, e),
                    "cx_px": circle.cx_px,
                    "cy_px": circle.cy_px,
                }

        return best_match if (best_match and best_distance < 1.0) else None

    # ------------------------------------------------------------------
    # 粗到位阶段 (位置控制 → PX4 原生飞往目标)
    # ------------------------------------------------------------------

    async def _do_transit(self, interface):
        """
        位置控制粗到位: 每帧更新位置 setpoint，持续飞向目标。
        距离 < ARRIVAL_THRESHOLD_M 后切换到速度 P 控制精调。

        ★ 方案 A: 每帧视觉检测 → 更新 setpoint，PX4 内部 200Hz+ 飞控执行。
        """
        if self._target is None:
            return ExecutionResult()

        # ---- 视觉检测：持续更新目标位置 ----
        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            match = self._match_target(interface, frame, alt)
            if match is not None:
                self._target.update(match)
                self._search_start = self.elapsed()

        except Exception as e:
            print(f"[粗到位] 视觉检测异常: {e}")
            return ExecutionResult()

        # ---- 检查是否到达目标附近 → 切速度精调 ----
        offset_north, offset_east = self._target["ned_offset"]
        if abs(offset_north) < ARRIVAL_THRESHOLD_M and abs(offset_east) < ARRIVAL_THRESHOLD_M:
            self.phase = "align"
            self._align_start_time = self.elapsed()
            print(f"[粗到位] 到达目标附近 "
                  f"(N={offset_north:.2f} E={offset_east:.2f})，切换速度 P 控制")
            return ExecutionResult()

        # ---- 更新位置 setpoint: 持续飞向目标 (offset 旋转到真北 NED) ----
        dn, de = self._rotate_offset(
            offset_north, offset_east, interface.FIELD_YAW_DEG)
        sp = PositionNedYaw(
            self._enter_position.north_m + dn,
            self._enter_position.east_m + de,
            -self._enter_alt,
            interface.FIELD_YAW_DEG,
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

        # ---- 对准阶段超时 ----
        if self.elapsed() - self._align_start_time > 20.0:
            self.error = "对准超时"
            print("[对准] 对准超时，退回 SearchState")
            interface.clear_velocity()
            return ExecutionResult(done=True)

        try:
            frame = await capture_frame_async()
            alt = await interface.get_altitude()

            match = self._match_target(interface, frame, alt)
            if match is not None:
                self._target.update(match)
                self._search_start = self.elapsed()

        except Exception as e:
            print(f"[对准] 视觉检测异常: {e}")
            return ExecutionResult()

        # ---- 检查对准精度 ----
        offset_north, offset_east = self._target["ned_offset"]
        if abs(offset_north) < ALIGN_THRESHOLD_M and abs(offset_east) < ALIGN_THRESHOLD_M:
            # ★ 切回位置模式，让 descend 阶段用位置控制下降
            interface.clear_velocity()
            self.phase = "descend"
            self._descend_start_time = self.elapsed()
            print("[对准] 对准成功，开始下降")
            return ExecutionResult()

        # ---- 发送速度指令 (PID 或 P 控制, offset 旋转到真北 NED) ----
        if self._pid_n is not None:
            vn = self._pid_n.update(0.0, offset_north)
        else:
            vn = VISUAL_SERVO_KP * offset_north

        if self._pid_e is not None:
            ve = self._pid_e.update(0.0, offset_east)
        else:
            ve = VISUAL_SERVO_KP * offset_east

        vn, ve = self._rotate_offset(vn, ve, interface.FIELD_YAW_DEG)
        vel = VelocityNedYaw(vn, ve, 0.0, interface.FIELD_YAW_DEG)
        interface.update_velocity_setpoint(vel)

        return ExecutionResult()

    # ------------------------------------------------------------------
    # 下降阶段
    # ------------------------------------------------------------------

    async def _do_descend(self, interface):
        """垂直下降到投放高度，同时视觉跟踪更新水平位置。"""
        alt = await interface.get_altitude()

        if alt <= DROP_ALIGN_ALTITUDE_M:
            self.phase = "drop"
            print(f"[下降] 到达投放高度: {alt:.2f}m")
            return ExecutionResult()

        if self.elapsed() - self._descend_start_time > 15.0:
            self.error = "下降超时"
            print("[下降] 下降超时，退出")
            return ExecutionResult(done=True)

        if self._target is None:
            return ExecutionResult()

        # ---- 视觉检测：下降过程中持续跟踪目标 ----
        try:
            frame = await capture_frame_async()

            match = self._match_target(interface, frame, alt)
            if match is not None:
                self._target.update(match)
                self._search_start = self.elapsed()

        except Exception as e:
            print(f"[下降] 视觉检测异常: {e}")

        # ---- 更新位置 setpoint ----
        offset_north, offset_east = self._target["ned_offset"]
        dn, de = self._rotate_offset(
            offset_north, offset_east, interface.FIELD_YAW_DEG)
        sp = PositionNedYaw(
            self._enter_position.north_m + dn,
            self._enter_position.east_m + de,
            -DROP_ALIGN_ALTITUDE_M,
            interface.FIELD_YAW_DEG,
        )
        interface.update_setpoint(sp)

        return ExecutionResult()

    # ------------------------------------------------------------------
    # 投桶阶段
    # ------------------------------------------------------------------

    async def _do_drop(self, interface):
        """执行投桶动作，更新 shared["goal"]。"""
        if self._drop_done:
            self.phase = "return_home"
            return ExecutionResult()

        # ---- 1. 执行投桶动作 ----
        print("[投桶] 执行投桶...")
        if USE_SERVO:
            await interface.set_actuator(DROP_SERVO_CHANNEL, 1.0)
            await asyncio.sleep(0.5)
            await interface.set_actuator(DROP_SERVO_CHANNEL, -1.0)
        else:
            print("[投桶] 舵机已禁用 (USE_SERVO=False)，模拟投桶完成")

        self._drop_done = True
        print("[投桶] 投桶完成")

        # ---- 2. 更新共享 goal 标志 ----
        goal = interface.shared.get("goal")
        if goal is not None:
            if self.target_type == "15":
                goal[0] = 1
                print("[ROLL_TASK] 标记 15cm 目标为已投放")
            else:  # "20"
                goal[1] = 1
                print("[ROLL_TASK] 标记 20cm 目标为已投放")
        else:
            print("[ROLL_TASK] 警告: shared['goal'] 未初始化，跳过更新")

        # ---- 3. 进入返回阶段 ----
        self.phase = "return_home"
        return ExecutionResult()

    # ------------------------------------------------------------------
    # 返回阶段
    # ------------------------------------------------------------------

    async def _do_return_home(self, interface):
        """返回进入位置，然后 done=True 弹出自身。"""
        current_pos = await interface.get_position_ned()

        dx = current_pos.north_m - self._enter_position.north_m
        dy = current_pos.east_m - self._enter_position.east_m
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance < 0.3:
            if self._retrying:
                # 视觉丢失重试：回到起点，重新检测目标
                self._retrying = False
                self._search_start = self.elapsed()
                print("[返回] 已回到 POSITION_LEFT，重新检测目标...")
                try:
                    frame = await capture_frame_async()
                    alt = await interface.get_altitude()
                    # 重新检测匹配 self.target_type 的目标
                    results = self.pipeline.process_frame(frame, alt_rel_m=alt)
                    valid = [r for r in results if r["edge_success"]]
                    target_match = None
                    for r in valid:
                        d = r["diameter_m"] * 100
                        if self.target_type == "15" and abs(d - 15) < 2:
                            target_match = r; break
                        elif self.target_type == "20" and abs(d - 20) < 2:
                            target_match = r; break
                    if target_match is None:
                        self.error = "重试失败：未检测到目标"
                        print("[返回] 重试失败，退出")
                        return ExecutionResult(done=True)
                    self._target = {
                        "cx_px": target_match["circle"].cx_px,
                        "cy_px": target_match["circle"].cy_px,
                        "diameter_m": target_match["diameter_m"],
                        "ned_offset": (0.0, 0.0),
                    }
                    fx = self.pipeline.circle.fx; fy = self.pipeline.circle.fy
                    cx = self.pipeline.circle.cx; cy = self.pipeline.circle.cy
                    z_c = alt - BUCKET_HEIGHT_M
                    e = (self._target["cx_px"] - cx) * z_c / fx
                    n = -(self._target["cy_px"] - cy) * z_c / fy
                    self._target["ned_offset"] = (n, e)
                    self.phase = "transit"
                    print("[返回] 重试：重新进入粗到位阶段")
                    return ExecutionResult()
                except Exception as e:
                    self.error = f"重试视觉检测失败: {e}"
                    print(f"[返回] {self.error}")
                    return ExecutionResult(done=True)

            self.is_completed = True
            goal = interface.shared.get("goal")
            if goal == [1, 1]:
                interface.shared["goto_recon"] = True
                print("[返回] 已回到 POSITION_LEFT，全部投完，通知 SearchState 切 ReconState")
            else:
                print("[返回] 已回到 POSITION_LEFT，弹出自身 → SearchState resume")
            return ExecutionResult(done=True)

        sp = PositionNedYaw(
            self._enter_position.north_m,
            self._enter_position.east_m,
            -self._enter_alt,
            interface.FIELD_YAW_DEG,
        )
        interface.update_setpoint(sp)
        return ExecutionResult()

    # ------------------------------------------------------------------
    # 工具：场地偏移 → 真北 NED 旋转
    # ------------------------------------------------------------------

    @staticmethod
    def _rotate_offset(offset_north: float, offset_east: float,
                        yaw_deg: float):
        """场地坐标系的偏移 → 真北 NED 偏移"""
        theta = math.radians(yaw_deg)
        dn = offset_north * math.cos(theta) - offset_east * math.sin(theta)
        de = offset_north * math.sin(theta) + offset_east * math.cos(theta)
        return dn, de
