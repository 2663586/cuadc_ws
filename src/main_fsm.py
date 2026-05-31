"""
Mission finite-state-machine engine.

Manages the mission queue, cycles through states, performs
global health checks every iteration, and handles errors / timeouts.
"""

import asyncio
import time
from typing import Optional

from mavsdk.offboard import PositionNedYaw

from interface import PX4Interface
from config import FSM_LOOP_HZ
from states.base_state import BaseState
from states.takeoff import TakeoffState
from states.hover import HoverState
from states.transit import TransitState
from states.search import SearchState
from states.align import AlignState
from states.drop import DropState
from states.recon import ReconState
from states.land import PrecisionLandState


class MissionFSM:
    """Mission state-machine engine."""

    def __init__(self, interface: PX4Interface):
        self.interface = interface
        self.mission_queue: list[BaseState] = []
        self.current_state: Optional[BaseState] = None
        self.state_index = 0

        # Register the unhealthy callback
        interface._on_unhealthy = self._handle_unhealthy

    def build_mission(self):
        """Build the mission sequence."""
        from config import (
            CRUISE_ALTITUDE_M, DROP_ZONE_DISTANCE_M, RECON_ZONE_DISTANCE_M,
            LAND_START_ALTITUDE_M, TRANSIT_SPEED_MPS,
        )

        self.mission_queue = [
            TakeoffState(target_alt=CRUISE_ALTITUDE_M, timeout_s=30),
            HoverState(hover_time=1.0),

            TransitState(x=DROP_ZONE_DISTANCE_M, y=0.0, z=CRUISE_ALTITUDE_M,
                         speed=TRANSIT_SPEED_MPS, timeout_s=30),
            HoverState(hover_time=1.0),

            # Drop phase: coarse detection once, then align + drop per bottle
            SearchState(timeout_s=30),
            AlignState(bottle_index=1, timeout_s=60),
            DropState(bottle_index=1, timeout_s=10),
            AlignState(bottle_index=2, timeout_s=60),
            DropState(bottle_index=2, timeout_s=10),

            TransitState(x=RECON_ZONE_DISTANCE_M, y=0.0, z=CRUISE_ALTITUDE_M,
                         speed=TRANSIT_SPEED_MPS, timeout_s=30),
            HoverState(hover_time=1.0),
            ReconState(timeout_s=120),

            TransitState(x=0.0, y=0.0, z=LAND_START_ALTITUDE_M,
                         speed=TRANSIT_SPEED_MPS, timeout_s=60),
            HoverState(hover_time=1.0),
            PrecisionLandState(timeout_s=60),
        ]
        self.state_index = 0

    async def run(self):
        """Main FSM loop."""
        self.build_mission()
        await self.interface.connect_and_setup()
        await self.interface.arm_and_offboard()

        while self.state_index < len(self.mission_queue):
            state = self.mission_queue[self.state_index]
            self.current_state = state

            # ---- Global health check ----
            if not await self.interface.global_guard_check():
                await self._handle_unhealthy("global_guard failed before state")
                break

            # ---- Enter state ----
            await state.enter(self.interface)

            # ---- Execute loop ----
            while True:
                if not await self.interface.global_guard_check():
                    await self._handle_unhealthy("health check in state")
                    return

                try:
                    done, _ = await state.execute(self.interface)
                except Exception as e:
                    state.error = str(e)
                    await self._handle_state_error(state, e)
                    break

                if done:
                    break
                await asyncio.sleep(1.0 / FSM_LOOP_HZ)

            # ---- Exit state ----
            await state.exit(self.interface)

            # ---- Timeout handling ----
            if state.is_timed_out() and not state.is_completed:
                print(f"[WARN] {state.name} timed out ({state.timeout_s}s), "
                      f"skipping")

            self.state_index += 1

        # Mission complete
        await self.interface.disarm()
        print("[INFO] Mission complete")

    async def _handle_unhealthy(self, reason: str):
        """Emergency: force hover on health failure."""
        print(f"[EMERGENCY] Global health check failed: {reason}")
        self.interface.update_setpoint(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        )

    async def _handle_state_error(self, state: BaseState, error: Exception):
        """Unified error handling for state execution failures."""
        print(f"[ERROR] {state.name} raised exception: {error}")
        self.interface.update_setpoint(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        )
