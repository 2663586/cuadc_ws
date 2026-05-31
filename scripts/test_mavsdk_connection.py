#!/usr/bin/env python3
"""
Quick MAVSDK-to-PX4-SITL connection test.

Run AFTER starting PX4 SITL (e.g. `make px4_sitl jmavsim`).
This script connects, waits for GPS, arms, takes off, hovers, then lands.

Usage:
    python3 scripts/test_mavsdk_connection.py
    python3 scripts/test_mavsdk_connection.py --address udp://:14540
"""

import asyncio
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from mavsdk import System
from mavsdk.offboard import PositionNedYaw


async def test_connection(system_address: str = "udp://:14540"):
    print(f"Connecting to PX4 at {system_address} ...")
    drone = System()
    await drone.connect(system_address=system_address)

    # Wait for connection
    print("Waiting for connection ...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("  Connected.")
            break

    # Wait for GPS and home position
    print("Waiting for GPS lock and home position ...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("  GPS OK, home position set.")
            break

    # Arm
    print("Arming ...")
    await drone.action.arm()
    print("  Armed.")

    # Enter offboard mode
    print("Entering offboard mode ...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()
    print("  Offboard mode active.")

    # Take off to 5 m
    print("Taking off to 5 m ...")
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()
    await asyncio.sleep(8)

    # Hover at 5 m for 3 seconds
    print("Hovering at 5 m ...")
    for _ in range(30):
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -5.0, 0.0)
        )
        await asyncio.sleep(0.1)

    # Fly forward 10 m
    print("Flying forward 10 m ...")
    for _ in range(50):
        await drone.offboard.set_position_ned(
            PositionNedYaw(10.0, 0.0, -5.0, 0.0)
        )
        await asyncio.sleep(0.1)

    # Fly back to origin
    print("Returning to origin ...")
    for _ in range(50):
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -5.0, 0.0)
        )
        await asyncio.sleep(0.1)

    # Land
    print("Landing ...")
    await drone.offboard.stop()
    await drone.action.land()
    await asyncio.sleep(5)

    # Disarm
    await drone.action.disarm()
    print("=== Test complete: chain works! ===")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="udp://:14540")
    args = parser.parse_args()

    asyncio.run(test_connection(args.address))
