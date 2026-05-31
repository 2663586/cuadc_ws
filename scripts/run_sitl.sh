#!/bin/bash
# ------------------------------------------------------------------
# Launch PX4 SITL with jMAVSim (lightweight, no Gazebo needed).
#
# This starts PX4 in SITL mode and exposes MAVLink on:
#   UDP port 14540 (offboard API) — our MAVSDK program connects here
#   UDP port 14550 (ground station) — QGroundControl
#   TCP port 4560  (simulation control)
#
# The simulated vehicle starts at the origin.
#
# Usage:
#   bash scripts/run_sitl.sh          # default: jMAVSim quadcopter
#   bash scripts/run_sitl.sh gz_x500  # Gazebo-based simulation
# ------------------------------------------------------------------

set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SIM_TARGET="${1:-jmavsim}"

echo "=== Launching PX4 SITL ($SIM_TARGET) ==="

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_DIR"
    echo "Clone it first:"
    echo "  cd ~ && git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.16.1 --depth 1"
    exit 1
fi

# Build and run
cd "$PX4_DIR"
echo "Building PX4 SITL target: $SIM_TARGET ..."
make "px4_sitl" "$SIM_TARGET"
