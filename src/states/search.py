"""
搜索状态 —— 矩形航线巡逻 + VisionPipeline 检测 + 目标匹配。

飞行模式：
  沿矩形航线四顶点（SEARCH_WAYPOINTS）循环飞行，每帧执行
  VisionPipeline 检测（YOLO → Canny 边缘 → HoughCircles → 针孔直径）。
  检测到目标圆柱体后，匹配直径判断瓶型（15cm / 20cm），
  将目标信息存入 interface.shared，通过栈式抢占切入 AlignState。

  对准完成后 resume 继续巡逻搜索。

目标匹配逻辑：
  - 15cm 瓶（goal[0]）：abs(diameter_cm - 15) ≤ EPSILON_DIAMETER_CM
  - 20cm 瓶（goal[1]）：abs(diameter_cm - 20) ≤ EPSILON_DIAMETER_CM

坐标系：
  图像上方 = 场地北，图像右方 = 场地东
  pixel_to_ned_offset 将圆心像素坐标转换为场地 NED 偏移量。
"""

from dataclasses import dataclass
from typing import Tuple, TYPE_CHECKING

from .base_state import BaseState, ExecutionResult
from .align import AlignState
from config import (CRUISE_ALTITUDE_M, ARRIVAL_THRESHOLD_M,
                    EPSILON_DIAMETER_CM, SEARCH_WAYPOINTS)
from vision.camera import capture_frame_async
from vision.circle_detector import DEFAULT_CAMERA_MATRIX
from vision.pipeline import VisionPipeline

if TYPE_CHECKING:
    from interface import PX4Interface

# 相机内参 — 从 CircleDetector 提取，避免硬编码不同步
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
    dx = cx_px - cx          # 右 = 东
    dy = cy_px - cy          # 下 = 南
    east_m = dx * z_c / fx
    north_m = -dy * z_c / fy  # 下 = 南 = -北
    return (north_m, east_m)


# ---------------------------------------------------------------------------
# 数据桥接：VisionPipeline 检测结果 → AlignState
# ---------------------------------------------------------------------------

@dataclass
class CylinderTarget:
    """SearchState 检测目标，AlignState 通过 .ned_offset[0/1] 消费。"""
    ned_offset: Tuple[float, float]  # (north_m, east_m)
    diameter_m: float
    conf: float
    circle_cx_px: int
    circle_cy_px: int


class SearchState(BaseState):

    def __init__(self, timeout_s: float = 120):
        super().__init__("Search", timeout_s)
        self.goal = [0, 0]          # 0=未找到, 1=已找到
        self._rect_waypoints = []   # 矩形航线航点列表
        self._wp_index = 0

        # 视觉流水线: YOLO → Canny边缘 → HoughCircles → 直径
        self.pipeline = VisionPipeline(
            model_path="models/yolov11n_800_best_FP16.engine",
            yolo_conf=0.5,
            circle_conf_threshold=0.3,
        )

    # ------------------------------------------------------------------
    async def enter(self, interface: "PX4Interface"):
        await super().enter(interface)

        # 矩形航线四顶点 — 从 config.SEARCH_WAYPOINTS 读取
        # 每个航点: (north_m, east_m)
        self._rect_waypoints = list(SEARCH_WAYPOINTS)
        self._wp_index = 0

        print(f"[搜索] 矩形航线开始，{len(self._rect_waypoints)} 个航点，"
              f"高度 {CRUISE_ALTITUDE_M:.1f}m")

    # ------------------------------------------------------------------
    async def execute(self, interface: "PX4Interface"):
        # ---- 超时 ----
        if self.is_timed_out():
            self.error = "搜索超时"
            return ExecutionResult(done=True)

        # ---- 飞到当前航点（速度控制） ----
        arrived = await self._fly_to_target(interface, self._wp_index)

        # ---- 执行检测 ----
        alt = await interface.get_altitude()
        frame = await capture_frame_async()

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
                self._save_detection(interface, bottle=1, result=r, alt_m=alt)
                # 栈式抢占：挂起搜索 → 压入对准 → 对准完成后 resume 继续搜索
                return ExecutionResult(interrupt=AlignState(bottle_index=1))
            # 匹配 20cm 瓶 (goal[1])
            elif abs(diameter_cm - 20) <= EPSILON_DIAMETER_CM and self.goal[1] == 0:
                self.goal[1] = 1
                self._save_detection(interface, bottle=2, result=r, alt_m=alt)
                return ExecutionResult(interrupt=AlignState(bottle_index=2))

        if not arrived:
            return ExecutionResult()     # 还在路上，下一帧继续飞

        # ---- 无目标 → 推进到下一个航点，绕圈循环 ----
        self._wp_index = (self._wp_index + 1) % len(self._rect_waypoints)
        return ExecutionResult()

    # ------------------------------------------------------------------
    async def _fly_to_target(self, interface: "PX4Interface",
                              wp_idx: int) -> bool:
        """
        直接发送位置指令飞向航点 wp_idx。

        update_setpoint 同步心跳 + set_position_ned 立即生效，
        两条路径发同一目标，不产生模式切换冲突。
        """
        north_m, east_m = self._rect_waypoints[wp_idx]
        target = interface.field_to_ned(north_m, east_m, CRUISE_ALTITUDE_M)

        # 双写：心跳目标 + 直接发送
        interface.update_setpoint(target)
        try:
            await interface.drone.offboard.set_position_ned(target)
        except Exception as e:
            print(f"[搜索] set_position_ned 发送异常: {e}")

        # 到达判定
        pos = await interface.get_position_ned()
        dn = target.north_m - pos.north_m
        de = target.east_m - pos.east_m
        dist = (dn**2 + de**2) ** 0.5
        return dist < ARRIVAL_THRESHOLD_M

    # ------------------------------------------------------------------
    def _save_detection(self, interface: "PX4Interface", bottle: int,
                         result: dict, alt_m: float):
        """
        将检测结果写入 interface.shared，供 AlignState.enter() 读取。

        VisionPipeline 结果 → 像素转 NED → CylinderTarget →
        shared["drop_targets"][bottle-1]

        AlignState 期望:
            shared["drop_targets"] = [target1_or_None, target2_or_None]
            target.ned_offset[0]  → 场地北偏移 (米)
            target.ned_offset[1]  → 场地东偏移 (米)
        """
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
