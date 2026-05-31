#!/bin/bash
# ------------------------------------------------------------------
# PX4 SITL dependency installer for Ubuntu 24.04+
#
# Run this ONCE before first SITL launch:
#   bash scripts/setup_px4_deps.sh
#
# Installs: required system packages for PX4 SITL build + Gazebo.
# You may need sudo for apt installs; run with `sudo` if required.
# ------------------------------------------------------------------

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== Installing PX4 SITL system dependencies ==="

# --- Required packages for PX4 SITL compilation ---
# Core build tools
sudo apt-get update
sudo apt-get install -y \
    git cmake make gcc g++ python3-pip python3-venv \
    python3-dev python3-numpy python3-empy python3-jinja2 \
    python3-toml python3-packaging python3-yaml \
    ccache ninja-build

# --- PX4 SITL with Gazebo ---
# Install Gazebo packages (Harmonic for Ubuntu 24.04+)
sudo apt-get install -y \
    gz-harmonic \
    ros-humble-ros-gzharmonic 2>/dev/null || true

# Alternative: Gazebo Classic (if Harmonic not available)
# sudo apt-get install -y gazebo libgazebo-dev

echo ""
echo "=== Dependencies installed ==="
echo "Next: build PX4 SITL with:"
echo "  cd ~/PX4-Autopilot && make px4_sitl"
echo ""
echo "Or for Gazebo:"
echo "  cd ~/PX4-Autopilot && make px4_sitl gz_x500"
