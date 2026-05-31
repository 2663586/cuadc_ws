"""
PX4 communication layer — wraps all MAVSDK interactions.

Provides:
- Health watchdog (global guard) for fail-safe
- Offboard heartbeat loop (independent asyncio task, 20 Hz)
- Telemetry monitoring (health, battery, connection state)
- Field-to-NED coordinate conversion
- High-level commands (takeoff, land, set_actuator)
"""

import asyncio
import math
from dataclasses import dataclass
from typing import Optional, Callable

from mavsdk import System
from mavsdk.offboard import PositionNedYaw


@dataclass
class HealthStatus:
    """Snapshot of flight-controller health."""

    is_connected: bool = False
    is_armed: bool = False
    is_offboard: bool = False
    is_global_position_ok: bool = False
    is_home_position_ok: bool = False
    battery_pct: float = 100.0
    estimator_flags_ok: bool = True
    gps_fix_type: int = 0

    @property
    def is_healthy(self) -> bool:
        from config import BATTERY_LOW_THRESHOLD_PCT, GPS_FIX_MIN

        return (
            self.is_connected
            and self.is_armed
            and self.is_offboard
            and self.is_global_position_ok
            and self.is_home_position_ok
            and self.battery_pct > BATTERY_LOW_THRESHOLD_PCT
            and self.estimator_flags_ok
            and self.gps_fix_type >= GPS_FIX_MIN
        )


class PX4Interface:
    """Encapsulates all MAVSDK interactions with built-in safety mechanisms."""

    def __init__(self, system_address: str = "udp://:14540",
                 on_unhealthy: Optional[Callable] = None):
        self.drone = System()
        self.system_address = system_address
        self.health = HealthStatus()
        self._on_unhealthy = on_unhealthy

        # Heartbeat state
        self._last_setpoint = PositionNedYaw(0.0, 0.0, 0.0, 0.0)

        # Field yaw — auto-detected at arm time
        self.FIELD_YAW_DEG: float = 0.0

        # Shared storage for cross-state data (e.g. search results)
        self.shared: dict = {}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def connect_and_setup(self):
        """Connect to PX4, wait for GPS lock, detect field orientation."""
        await self.drone.connect(system_address=self.system_address)

        # Wait for connection
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break

        # Wait for global position and home position
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break

        # Auto-detect field orientation from current heading
        async for heading in self.drone.telemetry.heading():
            self.FIELD_YAW_DEG = heading.heading_deg
            print(f"[INFO] Field orientation auto-detected: "
                  f"FIELD_YAW_DEG = {self.FIELD_YAW_DEG:.1f} deg")
            break

        # Start background monitoring tasks
        asyncio.create_task(self._telemetry_watcher())
        asyncio.create_task(self._battery_watcher())
        asyncio.create_task(self._heartbeat_loop())

    async def arm_and_offboard(self):
        """Arm the drone and switch to offboard mode."""
        # Send a zero setpoint so offboard has something to latch onto
        await self.drone.offboard.set_position_ned(self._last_setpoint)
        await self.drone.action.arm()
        await self.drone.offboard.start()
        print("[INFO] Armed and offboard mode engaged")

    async def disarm(self):
        """Exit offboard mode and disarm."""
        await self.drone.offboard.stop()
        await self.drone.action.disarm()
        print("[INFO] Disarmed")

    # ------------------------------------------------------------------
    # Health watchdog
    # ------------------------------------------------------------------

    async def _telemetry_watcher(self):
        """Continuously update health snapshot from telemetry."""
        async for health in self.drone.telemetry.health():
            self.health.is_global_position_ok = health.is_global_position_ok
            self.health.is_home_position_ok = health.is_home_position_ok

        # Monitor status text for estimator anomalies
        async for status_text in self.drone.telemetry.status_text():
            if "estimator" in status_text.text.lower():
                if self._on_unhealthy:
                    self._on_unhealthy(status_text.text)

    async def _battery_watcher(self):
        """Continuously update battery percentage."""
        async for battery in self.drone.telemetry.battery():
            self.health.battery_pct = battery.remaining_percent

    async def global_guard_check(self) -> bool:
        """Called every loop cycle. Returns True if healthy."""
        # Update connection state
        async for state in self.drone.core.connection_state():
            self.health.is_connected = state.is_connected
            break

        # Update armed state
        async for armed in self.drone.telemetry.armed():
            self.health.is_armed = armed
            break

        # Update flight mode
        async for mode in self.drone.telemetry.flight_mode():
            self.health.is_offboard = (str(mode) == "OFFBOARD")
            break

        return self.health.is_healthy

    # ------------------------------------------------------------------
    # Offboard heartbeat (independent task — keeps PX4 from timing out)
    # ------------------------------------------------------------------

    async def _heartbeat_loop(self):
        """
        Send setpoint at fixed rate. If main logic hasn't updated the
        setpoint, re-send the last one (lazy hold).

        PX4 requires >= 2 Hz; we send at OFFBOARD_HEARTBEAT_HZ (~20 Hz)
        for margin.
        """
        from config import OFFBOARD_HEARTBEAT_HZ

        interval = 1.0 / OFFBOARD_HEARTBEAT_HZ
        while True:
            await self.drone.offboard.set_position_ned(self._last_setpoint)
            await asyncio.sleep(interval)

    def update_setpoint(self, setpoint: PositionNedYaw):
        """Main logic calls this to issue a new setpoint."""
        self._last_setpoint = setpoint

    # ------------------------------------------------------------------
    # High-level commands
    # ------------------------------------------------------------------

    async def takeoff(self, altitude_m: float):
        """Command takeoff to specified altitude."""
        await self.drone.action.set_takeoff_altitude(altitude_m)
        await self.drone.action.takeoff()
        print(f"[CMD] Takeoff to {altitude_m:.1f} m")

    async def set_actuator(self, index: int, value: float):
        """Control an actuator (e.g. servo) via AUX output."""
        await self.drone.action.set_actuator(index, value)
        print(f"[CMD] Actuator {index} -> {value:.2f}")

    async def land(self):
        """Command auto-land."""
        await self.drone.action.land()
        print("[CMD] Land")

    # ------------------------------------------------------------------
    # Telemetry queries (snapshot reads)
    # ------------------------------------------------------------------

    async def get_position_ned(self) -> PositionNedYaw:
        """Get current NED position (single snapshot)."""
        async for odom in self.drone.telemetry.odometry():
            return PositionNedYaw(
                odom.position_body.x_m,
                odom.position_body.y_m,
                odom.position_body.z_m,
                odom.position_body.heading_deg,
            )

    async def get_altitude(self) -> float:
        """Get current relative altitude in meters."""
        async for pos in self.drone.telemetry.position():
            return pos.relative_altitude_m

    async def get_heading(self) -> float:
        """Get current heading in degrees."""
        async for heading in self.drone.telemetry.heading():
            return heading.heading_deg

    # ------------------------------------------------------------------
    # Field-to-NED coordinate conversion
    # ------------------------------------------------------------------

    def field_to_ned(self, forward_m: float, right_m: float,
                     height_m: float) -> PositionNedYaw:
        """
        Convert field coordinates (forward / right / height) to NED + yaw.

        The N axis is fixed to true north. FIELD_YAW_DEG records the
        true-north bearing of the field's forward direction.

        Args:
            forward_m: distance along field forward axis (m)
            right_m:   distance along field right axis (m)
            height_m:  flight altitude (m, positive up)

        Returns:
            PositionNedYaw with north, east, down, and yaw set to field heading.
        """
        theta = math.radians(self.FIELD_YAW_DEG)
        north_m = forward_m * math.cos(theta) - right_m * math.sin(theta)
        east_m = forward_m * math.sin(theta) + right_m * math.cos(theta)
        return PositionNedYaw(north_m, east_m, -height_m, self.FIELD_YAW_DEG)
