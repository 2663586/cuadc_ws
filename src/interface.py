"""
PX4 通信层 —— 封装所有 MAVSDK 交互。

提供：
- 健康看门狗（全局守卫）用于故障安全
- Offboard 心跳循环（独立 asyncio 任务，20 Hz）
- 遥测监控（健康状态、电量、连接状态）
- 场地坐标到 NED 坐标的转换
- 高级指令（降落、舵机控制）
"""

import asyncio
import math
from dataclasses import dataclass
from typing import Optional, Callable

from mavsdk import System
from mavsdk.offboard import PositionNedYaw


@dataclass
class HealthStatus:
    """飞控健康状态快照。"""

    is_connected: bool = False
    is_armed: bool = False
    is_offboard: bool = False
    is_global_position_ok: bool = False
    is_home_position_ok: bool = False
    battery_pct: float = 100.0
    estimator_flags_ok: bool = True
    gps_fix_type: int = 3  # SITL 始终有 3D 定位；由 _gps_watcher 更新
    altitude_m: float = 0.0

    @property
    def is_healthy(self) -> bool:
        from config import BATTERY_LOW_THRESHOLD_PCT, GPS_FIX_MIN

        return (
            self.is_connected
            and self.is_armed
            and self.is_global_position_ok
            and self.is_home_position_ok
            and self.battery_pct > BATTERY_LOW_THRESHOLD_PCT
            and self.estimator_flags_ok
            and self.gps_fix_type >= GPS_FIX_MIN
        )


class PX4Interface:
    """封装所有 MAVSDK 交互，内置安全机制。"""

    def __init__(self, system_address: str = "udp://0.0.0.0:14540",
                 on_unhealthy: Optional[Callable] = None):
        self.drone = System()
        self.system_address = system_address
        self.health = HealthStatus()
        self._on_unhealthy = on_unhealthy

        # 心跳状态
        self._last_setpoint = PositionNedYaw(0.0, 0.0, 0.0, 0.0)

        # 场地航向 —— 上锁时自动检测
        self.FIELD_YAW_DEG: float = 0.0

        # 跨状态共享数据存储（例如搜索结果）
        self.shared: dict = {}

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def connect_and_setup(self):
        """连接到 PX4，等待 GPS 锁定，检测场地朝向。"""
        await self.drone.connect(system_address=self.system_address)

        # 等待连接
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break
        self.health.is_connected = True
        print("[信息] 已连接到飞控")

        # 等待全局位置和家点位置
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
        self.health.is_global_position_ok = True
        self.health.is_home_position_ok = True
        print("[信息] GPS 已锁定，家点位置已记录")

        # 从当前航向自动检测场地朝向
        async for heading in self.drone.telemetry.heading():
            self.FIELD_YAW_DEG = heading.heading_deg
            print(f"[信息] 场地朝向自动检测: "
                  f"FIELD_YAW_DEG = {self.FIELD_YAW_DEG:.1f} 度")
            break

        # 启动后台任务
        asyncio.create_task(self._heartbeat_loop())

    async def arm_and_offboard(self):
        """上锁并切换到 offboard 模式。"""
        # 发送零设定值以便 offboard 有东西可锁定
        await self.drone.offboard.set_position_ned(self._last_setpoint)
        await self.drone.action.arm()
        await self.drone.offboard.start()
        self.health.is_armed = True
        self.health.is_offboard = True
        print("[信息] 已上锁，offboard 模式已启用")

    async def disarm(self):
        """退出 offboard 模式并断开上锁。"""
        try:
            await self.drone.offboard.stop()
        except Exception:
            pass
        try:
            await self.drone.action.disarm()
        except Exception:
            pass
        print("[信息] 已断开上锁")

    # ------------------------------------------------------------------
    # 健康看门狗
    # ------------------------------------------------------------------

    async def global_guard_check(self) -> bool:
        """
        每个循环周期调用。返回 True 表示健康。

        所有遥测数据通过即时查询读取，节流至约 2 Hz
        以避免压垮 MAVSDK 的 gRPC 回调队列。
        两次读取之间返回上次缓存的结果。
        """
        import time as _time

        now = _time.monotonic()
        if not hasattr(self, "_last_guard_read"):
            self._last_guard_read = 0.0
        if not hasattr(self, "_cached_healthy"):
            self._cached_healthy = True

        # 节流：实际 MAVSDK 读取仅约 2 Hz
        if now - self._last_guard_read > 0.5:
            self._last_guard_read = now
            try:
                async for state in self.drone.core.connection_state():
                    self.health.is_connected = state.is_connected
                    break
                async for armed in self.drone.telemetry.armed():
                    self.health.is_armed = armed
                    break
                async for health in self.drone.telemetry.health():
                    self.health.is_global_position_ok = health.is_global_position_ok
                    self.health.is_home_position_ok = health.is_home_position_ok
                    break
                async for gps in self.drone.telemetry.gps_info():
                    self.health.gps_fix_type = gps.fix_type.value
                    break
                async for battery in self.drone.telemetry.battery():
                    self.health.battery_pct = battery.remaining_percent
                    break
                async for pos in self.drone.telemetry.position():
                    self.health.altitude_m = pos.relative_altitude_m
                    break
            except Exception as e:
                print(f"[调试] global_guard_check 读取错误: {e}")
            self._cached_healthy = self.health.is_healthy

        return self._cached_healthy

    # ------------------------------------------------------------------
    # Offboard 心跳（独立任务 —— 防止 PX4 超时）
    # ------------------------------------------------------------------

    async def _heartbeat_loop(self):
        """
        以固定频率发送设定值。如果主逻辑尚未更新设定值，
        则重新发送上一次的值（惰性保持）。

        PX4 要求 >= 2 Hz；我们以 OFFBOARD_HEARTBEAT_HZ（约 20 Hz）
        发送以留出余量。
        """
        from config import OFFBOARD_HEARTBEAT_HZ

        interval = 1.0 / OFFBOARD_HEARTBEAT_HZ
        while True:
            await self.drone.offboard.set_position_ned(self._last_setpoint)
            await asyncio.sleep(interval)

    def update_setpoint(self, setpoint: PositionNedYaw):
        """主逻辑调用此方法以发布新的设定值。"""
        self._last_setpoint = setpoint

    # ------------------------------------------------------------------
    # 高级指令
    # ------------------------------------------------------------------

    async def set_actuator(self, index: int, value: float):
        """通过 AUX 输出控制舵机（例如投放舵机）。"""
        await self.drone.action.set_actuator(index, value)
        print(f"[指令] 舵机 {index} -> {value:.2f}")

    async def land(self):
        """指令自动降落。（垂直下降）"""
        await self.drone.action.land()
        print("[指令] 降落")

    # ------------------------------------------------------------------
    # 遥测查询（快照读取）
    # ------------------------------------------------------------------

    async def get_position_ned(self) -> PositionNedYaw:
        """获取当前 NED 位置（单次快照）。"""
        async for odom in self.drone.telemetry.odometry():
            return PositionNedYaw(
                odom.position_body.x_m,
                odom.position_body.y_m,
                odom.position_body.z_m,
                odom.position_body.heading_deg,
            )

    async def get_altitude(self) -> float:
        """获取当前相对高度（缓存值，约 2 Hz 更新）。"""
        return self.health.altitude_m

    async def get_heading(self) -> float:
        """获取当前航向角度。"""
        async for heading in self.drone.telemetry.heading():
            return heading.heading_deg

    # ------------------------------------------------------------------
    # 场地坐标到 NED 坐标的转换
    # ------------------------------------------------------------------

    def field_to_ned(self, forward_m: float, right_m: float,
                     height_m: float) -> PositionNedYaw:
        """
        将场地坐标（前 / 右 / 高）转换为 NED + 航向。

        N 轴固定指向真北。FIELD_YAW_DEG 记录场地前方方向
        的真北方位角。

        参数:
            forward_m: 沿场地前方方向的距离（米）
            right_m:   沿场地右方方向的距离（米）
            height_m:  飞行高度（米，向上为正）

        返回:
            PositionNedYaw，包含 north、east、down 以及设为场地航向的 yaw。
        """
        theta = math.radians(self.FIELD_YAW_DEG)
        north_m = forward_m * math.cos(theta) - right_m * math.sin(theta)
        east_m = forward_m * math.sin(theta) + right_m * math.cos(theta)
        return PositionNedYaw(north_m, east_m, -height_m, self.FIELD_YAW_DEG)
