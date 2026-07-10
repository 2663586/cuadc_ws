"""
搜索状态 —— 矩形航线巡逻 → 发现桶 → 飞近验证 → 切入对准。

═══════════════════════════════════════════════════════════════
  流程
═══════════════════════════════════════════════════════════════

  patrol (巡逻):
    沿矩形航线四顶点循环飞行，每帧跑 VisionPipeline 检测。
    发现桶 → pixel_to_ned_offset 转 field 坐标 → 按距离排序
    → 去重 (_is_known_bucket: 距 P2/P3 < BUCKET_DEDUP_RADIUS_M 则跳过)
    → 取最近的新桶 → 记 return_pos=P1, phase="investigate", 飞向桶。

  investigate (抵近验证):
    飞向桶 (transit 风格, setpoint+心跳), 到达后调 Visual_Check_R()。
    匹配成功 → 记 P2, interrupt=AlignState(bottle_index)。
    匹配失败 → 记 P3, 飞回 P1, 到达后恢复 patrol。

═══════════════════════════════════════════════════════════════
  传给下游状态 (interface.shared)
═══════════════════════════════════════════════════════════════

  shared["drop_targets"]   = [CylinderTarget, CylinderTarget_or_None]
  shared["return_pos"]     = (north_m, east_m) 或 (-1234, -1234)
    → (-1234, -1234) 是哨兵值, 表示"这是第二个桶, 投完不需要返回 P1,
       直接切侦察"。RollTask/Align 读到以此值时应跳过 return_home。
  AlignState(bottle_index) = 1 或 2

═══════════════════════════════════════════════════════════════
  注意事项 (给 RollTask / Align 开发者)
═══════════════════════════════════════════════════════════════

  1. return_pos 为 sentinel(-1234,-1234) 时, 投完后不要返回 P1,
     应直接 done 退出, 由 SearchState resume 后切 ReconState。

  2. shared["drop_targets"] 中每个 CylinderTarget 的 ned_offset
     是该桶相对于无人机当前位置的 field 偏移 (米), 不是绝对坐标。
     AlignState/RollTask 使用时应叠加到自身当前位置上。

  3. P1/P2/P3 是 SearchState 内部变量, 不在 shared 中。
     下游状态如需返回点, 读 shared["return_pos"]。

  4. Visual_Check_R() 是占位函数, 目前仅用 patrol 时缓存的直径判断。
     RollTask 的 enter() 应自行重新检测 (YOLO + Circle) 获取精确定位。

  5. 坐标系: P1/P2/P3 和 _rect_waypoints 存的是 field NED (场地坐标系),
     经 interface.field_to_ned() 旋转为真北 NED 后发送给 PX4。
     _drone_field_pos() 做逆旋转 (true NED → field NED)。

  6. _returning_to_p1 标志: 拒桶后飞回 P1 的路上, 到达 P1 时不跑
     Visual_Check_R, 直接切回 patrol。下游状态不需要关心此标志。
"""

import math
from dataclasses import dataclass
from typing import Tuple, Optional, TYPE_CHECKING

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult
from .align import AlignState
from config import (
    CRUISE_ALTITUDE_M, FIELD_YAW_DEG,
    ARRIVAL_THRESHOLD_M, BUCKET_DEDUP_RADIUS_M,
    EPSILON_DIAMETER_CM, SEARCH_SPEED_MPS,
    SEARCH_RECT_HALF_N_M, SEARCH_RECT_HALF_E_M,
    SEARCH_RECT_CENTER_N_M, SEARCH_RECT_CENTER_E_M,
)
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

# RollTask 投第二个桶后不返回 P1 的标记值
_SENTINEL = (-1234.0, -1234.0)


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------

def pixel_to_ned_offset(cx_px: float, cy_px: float, alt_rel_m: float,
                        fx: float = _FX, fy: float = _FY,
                        cx: float = _CX, cy: float = _CY,
                        bucket_h: float = _BUCKET_HEIGHT_M) -> Tuple[float, float]:
    """下视相机像素坐标 → 场地 NED 偏移 (north_m, east_m)。"""
    z_c = alt_rel_m - bucket_h
    if z_c <= 0:
        return (0.0, 0.0)
    dx = cx_px - cx
    dy = cy_px - cy
    east_m = dx * z_c / fx
    north_m = -dy * z_c / fy
    return (north_m, east_m)


# 占位 — 后续替换为真实视觉代码
def Visual_Check_R(detected_diameter_m: float) -> bool:
    """临时: 仅根据 YOLO+Circle 的直径估算判断是否为目标桶。"""
    dia_cm = detected_diameter_m * 100
    return abs(dia_cm - 15) <= EPSILON_DIAMETER_CM or abs(dia_cm - 20) <= EPSILON_DIAMETER_CM


def _field_dist(a: PositionNedYaw, b: PositionNedYaw) -> float:
    """两个 field 坐标之间的水平距离 (m)。"""
    return math.hypot(a.north_m - b.north_m, a.east_m - b.east_m)


# ---------------------------------------------------------------------------
# 数据桥接
# ---------------------------------------------------------------------------

@dataclass
class CylinderTarget:
    ned_offset: Tuple[float, float]
    diameter_m: float
    conf: float
    circle_cx_px: int
    circle_cy_px: int


# ---------------------------------------------------------------------------
# SearchState
# ---------------------------------------------------------------------------

class SearchState(BaseState):

    def __init__(self, timeout_s: float = 120):
        super().__init__("Search", timeout_s)
        self._rect_waypoints: list[PositionNedYaw] = []
        self._wp_index = 0
        self._original_vel_max: Optional[float] = None

        # 两阶段控制
        self.phase: str = "patrol"          # "patrol" | "investigate"
        self.P1: Optional[PositionNedYaw] = None   # 返回点
        self.P2: Optional[PositionNedYaw] = None   # 已验证桶
        self.P3: Optional[PositionNedYaw] = None   # 已排除桶
        self._investigate_target: Optional[PositionNedYaw] = None
        self._investigate_diameter: float = -1.0   # patrol 时缓存的直径
        self._returning_to_p1: bool = False        # 拒桶后正在返回 P1

        # 视觉
        self.pipeline = None
        try:
            self.pipeline = VisionPipeline(
                model_path=None,
                yolo_conf=0.5,
                circle_conf_threshold=0.3,
            )
        except Exception as e:
            print(f"[搜索] VisionPipeline 初始化失败: {e}", flush=True)

    # ==================================================================
    # 生命周期
    # ==================================================================

    async def enter(self, interface: "PX4Interface"):
        await super().enter(interface)

        # ---- 复位两阶段状态 ----
        self.phase = "patrol"
        self.P1 = self.P2 = self.P3 = None
        self._investigate_target = None

        # ---- 矩形航线 ----
        cn = SEARCH_RECT_CENTER_N_M
        ce = SEARCH_RECT_CENTER_E_M
        hn = SEARCH_RECT_HALF_N_M
        he = SEARCH_RECT_HALF_E_M
        pts = [
            (cn - hn, ce + he),   # 后右
            (cn + hn, ce + he),   # 前右
            (cn + hn, ce - he),   # 前左
            (cn - hn, ce - he),   # 后左
        ]
        self._rect_waypoints = []
        for i, (n, e) in enumerate(pts):
            prev = pts[(i - 1) % len(pts)]
            dn, de = n - prev[0], e - prev[1]
            yaw = math.degrees(math.atan2(de, dn))
            self._rect_waypoints.append(
                PositionNedYaw(n, e, -CRUISE_ALTITUDE_M, yaw))

        self._wp_index = 0

        # ---- 启动位置飞行 ----
        target = interface.field_to_ned(self._rect_waypoints[0])
        self._original_vel_max = await interface.start_position_flight(
            target, SEARCH_SPEED_MPS)

        print(f"[搜索] 矩形航线开始 {len(self._rect_waypoints)} 航点 "
              f"@{SEARCH_SPEED_MPS:.1f}m/s", flush=True)

    async def execute(self, interface: "PX4Interface"):
        if self.is_timed_out():
            self.error = "搜索超时"
            return ExecutionResult(done=True)

        if self.phase == "patrol":
            return await self._do_patrol(interface)
        else:
            return await self._do_investigate(interface)

    async def exit(self, interface: "PX4Interface"):
        await interface.restore_cruise_speed(self._original_vel_max)
        await super().exit(interface)

    # ==================================================================
    # phase = "patrol"
    # ==================================================================

    async def _do_patrol(self, interface: "PX4Interface") -> ExecutionResult:
        # 1) 飞向当前航点
        arrived = await self._fly_to_wp(interface, self._wp_index)

        # 2) 视觉检测
        if self.pipeline is not None:
            try:
                alt = await interface.get_altitude()
                frame = await capture_frame_async()
                results = self.pipeline.process_frame(frame, alt_rel_m=alt)

                # 收集所有有效检测 → field 坐标
                detected: list[PositionNedYaw] = []
                best_diameter: dict[tuple, float] = {}  # (n,e) → diameter_m

                drone_field = await self._drone_field_pos(interface)

                for r in results:
                    if not r["edge_success"]:
                        continue
                    circle = r["circle"]
                    off_n, off_e = pixel_to_ned_offset(
                        circle.cx_px, circle.cy_px, alt)
                    bucket_pos = PositionNedYaw(
                        drone_field.north_m + off_n,
                        drone_field.east_m  + off_e,
                        0.0, 0.0,
                    )
                    key = (round(bucket_pos.north_m, 2), round(bucket_pos.east_m, 2))
                    detected.append(bucket_pos)
                    best_diameter[key] = r["diameter_m"]

                # 3) 按距离从近到远排序
                detected.sort(key=lambda p: _field_dist(drone_field, p))

                # 4) 逐个去重，取第一个新桶
                for bucket in detected:
                    if self._is_known_bucket(bucket):
                        continue          # P2 或 P3，跳过
                    # 新桶 → 记 P1，切 investigate
                    self.P1 = drone_field
                    self._investigate_target = bucket
                    self.phase = "investigate"
                    # 存直径供 Visual_Check_R 使用
                    key = (round(bucket.north_m, 2), round(bucket.east_m, 2))
                    self._investigate_diameter = best_diameter.get(key, -1)
                    print(f"[搜索] 发现新桶 @field({bucket.north_m:.1f},{bucket.east_m:.1f})",
                          flush=True)
                    # 立即发 setpoint 飞过去
                    self._fly_to_pos(interface, bucket)
                    return ExecutionResult()

            except Exception as e:
                print(f"[搜索] 视觉异常: {e}", flush=True)

        # 5) 未发现新桶
        if not arrived:
            return ExecutionResult()

        # 到达 → 推进航点
        self._wp_index = (self._wp_index + 1) % len(self._rect_waypoints)
        self._fly_to_wp(interface, self._wp_index)
        return ExecutionResult()

    # ==================================================================
    # phase = "investigate"
    # ==================================================================

    async def _do_investigate(self, interface: "PX4Interface") -> ExecutionResult:
        if self._investigate_target is None:
            self.phase = "patrol"
            return ExecutionResult()

        # 1) 飞向目标
        arrived = await self._fly_to_pos_arrived(interface, self._investigate_target)
        if not arrived:
            return ExecutionResult()

        # 2) 如果是拒桶后返回 P1 → 到达即恢复 patrol
        if self._returning_to_p1:
            print("[搜索] 已返回 P1，恢复巡逻", flush=True)
            self._returning_to_p1 = False
            self._investigate_target = None
            self.phase = "patrol"
            return ExecutionResult()

        # 3) 到达桶上方 → 验证直径
        dia_m = self._investigate_diameter
        ok = Visual_Check_R(dia_m) if dia_m > 0 else False

        if ok:
            # ---- 匹配成功 ----
            is_second_bottle = (self.P2 is not None)

            self.P2 = self._investigate_target
            print(f"[搜索] 验证通过 → P2=({self.P2.north_m:.1f},{self.P2.east_m:.1f})",
                  flush=True)

            return_pos = _SENTINEL if is_second_bottle else (
                (self.P1.north_m, self.P1.east_m) if self.P1 else _SENTINEL)

            if "drop_targets" not in interface.shared:
                interface.shared["drop_targets"] = [None, None]
            interface.shared["return_pos"] = return_pos

            bottle = 2 if is_second_bottle else 1
            self._investigate_target = None
            return ExecutionResult(interrupt=AlignState(bottle_index=bottle))

        else:
            # ---- 不匹配 → 标记 P3, 飞回 P1 ----
            self.P3 = self._investigate_target
            print(f"[搜索] 验证失败 → P3=({self.P3.north_m:.1f},{self.P3.east_m:.1f})",
                  flush=True)

            if self.P1 is not None:
                self._investigate_target = self.P1
                self._returning_to_p1 = True
                self._fly_to_pos(interface, self.P1)
            else:
                self.phase = "patrol"
            return ExecutionResult()

    # ==================================================================
    # 飞行辅助
    # ==================================================================

    async def _fly_to_wp(self, interface: "PX4Interface", wp_idx: int) -> bool:
        """飞向 _rect_waypoints[wp_idx]，返回是否到达。"""
        target = interface.field_to_ned(self._rect_waypoints[wp_idx])
        interface.update_setpoint(target)
        pos = await interface.get_position_ned()
        dist = math.hypot(target.north_m - pos.north_m,
                          target.east_m  - pos.east_m)
        return dist < ARRIVAL_THRESHOLD_M

    def _fly_to_pos(self, interface: "PX4Interface",
                    field_pos: PositionNedYaw):
        """发布 field 坐标为 setpoint（不阻塞）。"""
        target = interface.field_to_ned(field_pos)
        interface.update_setpoint(target)

    async def _fly_to_pos_arrived(self, interface: "PX4Interface",
                                   field_pos: PositionNedYaw) -> bool:
        """飞向任意 field 坐标，返回是否到达。"""
        target = interface.field_to_ned(field_pos)
        interface.update_setpoint(target)
        pos = await interface.get_position_ned()
        dist = math.hypot(target.north_m - pos.north_m,
                          target.east_m  - pos.east_m)
        return dist < ARRIVAL_THRESHOLD_M

    # ==================================================================
    # 坐标工具
    # ==================================================================

    async def _drone_field_pos(self, interface: "PX4Interface") -> PositionNedYaw:
        """当前无人机位置 → field NED 坐标。"""
        pos = await interface.get_position_ned()
        theta = math.radians(FIELD_YAW_DEG)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        # 逆旋转: true NED → field NED
        fn = pos.north_m * cos_t + pos.east_m * sin_t
        fe = -pos.north_m * sin_t + pos.east_m * cos_t
        return PositionNedYaw(fn, fe, pos.down_m, 0.0)

    # ==================================================================
    # 去重
    # ==================================================================

    def _is_known_bucket(self, pos: PositionNedYaw) -> bool:
        """桶坐标是否已存在于 P2 或 P3 附近。"""
        for known in (self.P2, self.P3):
            if known is not None and _field_dist(pos, known) < BUCKET_DEDUP_RADIUS_M:
                return True
        return False
