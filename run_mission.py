#!/usr/bin/env python3
"""
CUADC 2026 — Mission entry point.

Usage:
    python run_mission.py [--address udp://:14540] [--sim]

The program connects to PX4 (real or SITL), runs the full mission FSM,
and disarms on completion or emergency.
"""

import argparse
import asyncio
import sys
import os

# Ensure the src/ package is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from interface import PX4Interface
from main_fsm import MissionFSM
from logger_manager import LoggerManager


async def main(system_address: str = "udp://:14540", sim: bool = False):
    print("=" * 60)
    print("CUADC 2026 — Autonomous Mission Controller")
    print(f"PX4 address: {system_address}")
    print(f"Mode: {'Simulation' if sim else 'Live'}")
    print("=" * 60)

    # Initialise logger
    logger = LoggerManager()

    # Initialise communication layer
    interface = PX4Interface(system_address=system_address)
    fsm = MissionFSM(interface)

    try:
        logger.log_event("mission_start", sim=sim)
        await fsm.run()
        logger.log_event("mission_complete")
    except KeyboardInterrupt:
        print("\n[ABORT] Manual interrupt — emergency landing")
        logger.log_event("abort", reason="keyboard_interrupt")
        await interface.land()
    except Exception as e:
        print(f"\n[FATAL] Unhandled exception: {e}")
        logger.log_event("fatal", error=str(e))
        try:
            await interface.land()
        except Exception:
            pass
    finally:
        print("[INFO] Mission ended")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="CUADC 2026 autonomous mission controller"
    )
    parser.add_argument(
        "--address", default="udp://:14540",
        help="PX4 MAVLink address (default: udp://:14540)"
    )
    parser.add_argument(
        "--sim", action="store_true",
        help="Run in simulation mode"
    )
    args = parser.parse_args()

    asyncio.run(main(args.address, args.sim))
