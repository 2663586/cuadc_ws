"""
搜索状态 —— 矩形航线巡逻 + VisionPipeline 检测 + 任务调度。

飞行模式：
  沿矩形航线四顶点循环飞行，每帧执行 VisionPipeline 检测。
  检测到目标圆柱体后，将目标信息存入 interface.shared，
  通过栈式抢占依次切入：粗对准 → 精细对准 → 投放 → 返航。

每个子状态完成后 resume 回 SearchState，由 _next_action 决定下一步。
两轮投放完成后自动切入侦察，侦察完成后 SearchState 退出。
"""

import math
from dataclasses import dataclass
from typing import Tuple, TYPE_CHECKING

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult
from .align import AlignState
from .align_precise import AlignPreciseState
from .drop import DropState
from .post_drop_nav import PostDropNavState
from .recon import ReconState
from config import (CRUISE_ALTITUDE_M, ARRIVAL_THRESHOLD_M,
                    EPSILON_DIAMETER_CM,
                    SEARCH_SPEED_MPS,
                    SEARCH_RECT_HALF_N_M, SEARCH_RECT_HALF_E_M,
                    SEARCH_RECT_CENTER_N_M, SEARCH_RECT_CENTER_E_M)
from vision.camera import capture_frame_async
from vision.circle_detector import DEFAULT_CAMERA_MATRIX
from vision.pipeline import VisionPipeline

if TYPE_CHECKING:
    from interface import PX4Interface

# 相机内参
_FX = float(DEFAULT_CAMERA_MATRIX[0, 0])
_FY = float(DEFAULT_CAMERA_MATRIX[1, 1])
_CX = float(DEFAULT_CAMERA_MATRIX[0, 2])
_CY = float(DEFAULT_CAMERA_MATRIX[1, 2])
_BUCKET_HEIGHT_M = 0.30


# ---------------------------------------------------------------------------
# 工具：像素坐标 → 场地 NED 偏移
# ---------------------------------------------------------------------------

def pixel_to_ned_offset(cx_px: float, cy_px: float, alt_rel_m: float,
                        fx: float = _FX, fy: float = _FY,
                        cx: float = _CX, cy: float = _CY,
                        bucket_h: float = _BUCKET_HEIGHT_M) -> Tuple[float, float]:
    """下视相机像素坐标 → 场地 NED 偏移 (north_m, east_m)。

    图像上方=场地北，图像右方=场地东。"""
    z_c = alt_rel_m - bucket_h
    if z_c <= 0:
        return (0.0, 0.0)
    dx = cx_px - cx
    dy = cy_px - cy
    east_m = dx * z_c / fx
    north_m = -dy * z_c / fy
    return (north_m, east_m)


# ---------------------------------------------------------------------------
# 数据桥接
# ---------------------------------------------------------------------------

@dataclass
class CylinderTarget:
    """SearchState 检测目标，供 AlignState / AlignPreciseState 消费。"""
    ned_offset: Tuple[float, float]  # (north_m, east_m)
    diameter_m: float
    conf: float
    circle_cx_px: int
    circle_cy_px: int


class SearchState(BaseState):

    def __init__(self, timeout_s: float = 120):
        super().__init__("Search", timeout_s)
        self.goal = [0, 0]              # 0=未找到, 1=已找到并投放
        self._rect_waypoints = []
        self._wp_index = 0
        self._original_vel_max = None

        # 调度状态
        self._next_action = None        # None | "fine_align" | "drop" | "navigate"
        self._active_bottle = 0         # 当前正在处理的瓶子编号 (1 or 2)
        self._recon_triggered = False

        # 视觉流水线
        self.pipeline = VisionPipeline(
            model_path="models/yolov11n_800_best_FP16.engine",
            yolo_conf=0.5,
            circle_conf_threshold=0.3,
        )

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def enter(self, interface: "PX4Interface"):
        await super().enter(interface)

        # 初始化 shared 中的 goal（DropState 会更新它）
        interface.shared.setdefault("goal", self.goal)

        # ---- 矩形航线四顶点 ----
        cn = SEARCH_RECT_CENTER_N_M
        ce = SEARCH_RECT_CENTER_E_M
        hn = SEARCH_RECT_HALF_N_M
        he = SEARCH_RECT_HALF_E_M
        self._rect_waypoints = [
            (cn + hn, ce - he),  # 前左
            (cn + hn, ce + he),  # 前右
            (cn - hn, ce + he),  # 后右
            (cn - hn, ce - he),  # 后左
        ]
        self._wp_index = 0

        # ---- 启动 PX4 原生位置飞行 ----
        north_m, east_m = self._rect_waypoints[0]
        target = interface.field_to_ned(north_m, east_m, CRUISE_ALTITUDE_M)
        self._original_vel_max = await interface.start_position_flight(
            target, SEARCH_SPEED_MPS)

        print(f"[搜索] 矩形航线开始，{len(self._rect_waypoints)} 个航点，"
              f"高度 {CRUISE_ALTITUDE_M:.1f}m，速度 {SEARCH_SPEED_MPS:.1f} m/s",
              flush=True)

    async def suspend(self, interface: "PX4Interface"):
        """保存当前 true-NED 位置供投放后返航使用。"""
        pos = await interface.get_position_ned()
        interface.shared["position_left"] = PositionNedYaw(
            pos.north_m, pos.east_m, pos.down_m, interface.FIELD_YAW_DEG,
        )
        print(f"[搜索] 中断点已保存: "
              f"N({pos.north_m:.1f}) E({pos.east_m:.1f})")

    async def execute(self, interface: "PX4Interface"):
        # ---- 同步 goal（DropState 更新后会反映在这里） ----
        shared_goal = interface.shared.get("goal")
        if shared_goal is not None:
            self.goal = shared_goal

        # ---- 侦察已完成，任务结束 ----
        if self._recon_triggered and self.goal == [1, 1]:
            self.is_completed = True
            return ExecutionResult(done=True)

        # ---- 两轮投放完成 → 切入侦察 ----
        if self.goal == [1, 1] and not self._recon_triggered:
            self._recon_triggered = True
            print("[搜索] 两个目标均已投放，触发侦察")
            return ExecutionResult(interrupt=ReconState(timeout_s=120))

        # ---- 子阶段调度 ----
        if self._next_action == "fine_align":
            self._next_action = "drop"
            return ExecutionResult(interrupt=AlignPreciseState(
                bottle_index=self._active_bottle))
        elif self._next_action == "drop":
            self._next_action = "navigate"
            return ExecutionResult(interrupt=DropState(
                bottle_index=self._active_bottle))
        elif self._next_action == "navigate":
            self._next_action = None
            return ExecutionResult(interrupt=PostDropNavState())

        # ---- 超时 ----
        if self.is_timed_out():
            self.error = "搜索超时"
            return ExecutionResult(done=True)

        # ---- 飞到当前航点 ----
        arrived = await self._fly_to_target(interface, self._wp_index)

        # ---- 视觉检测 ----
        alt = await interface.get_altitude()
        frame = await capture_frame_async()
        results = self.pipeline.process_frame(frame, alt_rel_m=alt)

        for r in results:
            if not r["edge_success"]:
                continue

            diameter_cm = r["diameter_m"] * 100
            # 匹配 15cm 瓶 (goal[0])
            if abs(diameter_cm - 15) <= EPSILON_DIAMETER_CM and self.goal[0] == 0:
                self.goal[0] = 1
                self._save_detection(interface, bottle=1, result=r, alt_m=alt)
                self._next_action = "fine_align"
                self._active_bottle = 1
                return ExecutionResult(interrupt=AlignState(bottle_index=1))
            # 匹配 20cm 瓶 (goal[1])
            elif abs(diameter_cm - 20) <= EPSILON_DIAMETER_CM and self.goal[1] == 0:
                self.goal[1] = 1
                self._save_detection(interface, bottle=2, result=r, alt_m=alt)
                self._next_action = "fine_align"
                self._active_bottle = 2
                return ExecutionResult(interrupt=AlignState(bottle_index=2))

        if not arrived:
            return ExecutionResult()

        # ---- 推进航点 ----
        self._wp_index = (self._wp_index + 1) % len(self._rect_waypoints)
        north_m, east_m = self._rect_waypoints[self._wp_index]
        target = interface.field_to_ned(north_m, east_m, CRUISE_ALTITUDE_M)
        interface.update_setpoint(target)
        return ExecutionResult()

    async def exit(self, interface: "PX4Interface"):
        await interface.restore_cruise_speed(self._original_vel_max)
        await super().exit(interface)

    # ------------------------------------------------------------------
    # 航点飞行
    # ------------------------------------------------------------------

    async def _fly_to_target(self, interface: "PX4Interface",
                              wp_idx: int) -> bool:
        north_m, east_m = self._rect_waypoints[wp_idx]
        target = interface.field_to_ned(north_m, east_m, CRUISE_ALTITUDE_M)
        interface.update_setpoint(target)

        pos = await interface.get_position_ned()
        dn = target.north_m - pos.north_m
        de = target.east_m - pos.east_m
        dist = math.hypot(dn, de)
        return dist < ARRIVAL_THRESHOLD_M

    # ------------------------------------------------------------------
    # 检测结果 → 共享缓存
    # ------------------------------------------------------------------

    def _save_detection(self, interface: "PX4Interface", bottle: int,
                         result: dict, alt_m: float):
        circle = result["circle"]
        ned = pixel_to_ned_offset(circle.cx_px, circle.cy_px, alt_m)
        target = CylinderTarget(
            ned_offset=ned,
            diameter_m=result["diameter_m"],
            conf=result["det"]["conf"],
            circle_cx_px=circle.cx_px,
            circle_cy_px=circle.cy_px,
        )
        if "drop_targets" not in interface.shared:
            interface.shared["drop_targets"] = [None, None]
        interface.shared["drop_targets"][bottle - 1] = target
