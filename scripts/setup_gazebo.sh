#!/bin/bash
# ------------------------------------------------------------------
# Gazebo setup for PX4 SITL simulation.
#
# Attempts to install Gazebo. Requires sudo access.
# If sudo is not available, provides instructions for manual install.
#
# PX4 v1.16.1 supports:
#   - Gazebo Classic (gz-garden or earlier, via gazebo-classic target)
#   - Gazebo (via gz_x500 target)
#
# Usage:
#   bash scripts/setup_gazebo.sh
# ------------------------------------------------------------------

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

echo "============================================"
echo " CUADC 2026 — Gazebo Setup"
echo "============================================"
echo ""

install_via_sudo() {
    echo "Trying to install Gazebo via apt (requires sudo) ..."

    # Try Gazebo Harmonic (modern, for Ubuntu 24.04+)
    echo "Installing Gazebo Harmonic ..."
    sudo apt-get update
    sudo apt-get install -y gz-harmonic || {
        echo "Gazebo Harmonic not available, trying Gazebo Classic ..."
        # Fallback: Gazebo Classic
        sudo apt-get install -y gazebo libgazebo-dev || {
            echo "ERROR: Could not install Gazebo."
            echo "Please install manually following:"
            echo "  https://gazebosim.org/docs"
            return 1
        }
    }
    echo "Gazebo installed successfully."
}

install_manual_guide() {
    echo ""
    echo "============================================"
    echo " MANUAL GAZEBO INSTALLATION GUIDE"
    echo "============================================"
    echo ""
    echo "Option 1 — Gazebo Harmonic (recommended):"
    echo "  sudo apt-get update"
    echo "  sudo apt-get install gz-harmonic"
    echo ""
    echo "Option 2 — Gazebo Classic:"
    echo "  sudo apt-get update"
    echo "  sudo apt-get install gazebo libgazebo-dev"
    echo ""
    echo "Option 3 — Use Docker:"
    echo "  docker pull px4io/px4-dev-simulation-jammy"
    echo ""
    echo "After installing Gazebo, run:"
    echo "  cd $PX4_DIR"
    echo ""
    echo "  # For Gazebo Harmonic/Garden:"
    echo "  make px4_sitl gz_x500"
    echo ""
    echo "  # For Gazebo Classic:"
    echo "  make px4_sitl gazebo-classic"
    echo "============================================"
}

# Check if sudo is available
if command -v sudo &>/dev/null && sudo -n true 2>/dev/null; then
    install_via_sudo || install_manual_guide
else
    echo "WARNING: sudo not available or requires password."
    install_manual_guide
fi
