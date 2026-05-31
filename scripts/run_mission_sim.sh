#!/bin/bash
# ------------------------------------------------------------------
# Full simulation chain: PX4 SITL + MAVSDK mission program.
#
# Opens a tmux session with two panes:
#   Left:  PX4 SITL (jMAVSim by default)
#   Right: MAVSDK mission program (waits for SITL to be ready)
#
# Usage:
#   bash scripts/run_mission_sim.sh              # jMAVSim
#   bash scripts/run_mission_sim.sh gz_x500      # Gazebo
# ------------------------------------------------------------------

set -euo pipefail

SIM_TARGET="${1:-jmavsim}"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

SESSION="cuadc_sim"

# Kill existing session if present
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "=== Starting CUADC 2026 Simulation ==="
echo "Simulator: $SIM_TARGET"
echo "Project:   $PROJECT_DIR"
echo "PX4:       $PX4_DIR"
echo ""

# Create tmux session
tmux new-session -d -s "$SESSION" -n "sim"

# ---- Pane 0: PX4 SITL ----
tmux send-keys -t "$SESSION:0.0" \
    "cd $PX4_DIR && make px4_sitl $SIM_TARGET" C-m

# ---- Pane 1: Wait, then run mission ----
tmux split-window -h -t "$SESSION:0"
tmux send-keys -t "$SESSION:0.1" \
    "echo 'Waiting for PX4 SITL to start (10s)...' && sleep 10 && cd $PROJECT_DIR && python3 run_mission.py --sim" C-m

# Attach
tmux attach-session -t "$SESSION"
