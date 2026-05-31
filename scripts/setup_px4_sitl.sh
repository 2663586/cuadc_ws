#!/bin/bash
# ------------------------------------------------------------------
# PX4 SITL + Gazebo setup for CUADC 2026 simulation
#
# This script sets up the full simulation toolchain:
#   1. Downloads portable JDK (needed for jMAVSim)
#   2. Builds PX4 SITL binary
#   3. Installs Gazebo (if available)
#   4. Sets up MAVSDK Python environment
#
# Usage:
#   bash scripts/setup_px4_sitl.sh
# ------------------------------------------------------------------

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
JDK_DIR="$HOME/.local/jdk21"
GAZEBO_SETUP_SCRIPT="$PROJECT_DIR/scripts/setup_gazebo.sh"

echo "============================================"
echo " CUADC 2026 — SITL Environment Setup"
echo "============================================"
echo ""

# ---- Step 1: Portable JDK for jMAVSim ----
if [ ! -d "$JDK_DIR" ]; then
    echo "[1/3] Downloading portable JDK 21 ..."
    mkdir -p "$(dirname "$JDK_DIR")"
    cd /tmp
    curl -sL "https://api.adoptium.net/v3/binary/latest/21/ga/linux/x64/jdk/hotspot/normal/eclipse?project=jdk" \
        -o /tmp/jdk21.tar.gz
    mkdir -p "$JDK_DIR"
    tar xzf /tmp/jdk21.tar.gz -C "$JDK_DIR" --strip-components=1
    rm -f /tmp/jdk21.tar.gz
    echo "  JDK 21 installed at $JDK_DIR"
else
    echo "[1/3] JDK 21 already installed at $JDK_DIR"
fi

# Add JDK to PATH for this session and future use
export PATH="$JDK_DIR/bin:$PATH"
export JAVA_HOME="$JDK_DIR"
echo "export PATH=\"$JDK_DIR/bin:\$PATH\"" >> "$HOME/.bashrc"
echo "export JAVA_HOME=\"$JDK_DIR\"" >> "$HOME/.bashrc"

# ---- Step 2: Install Python deps ----
echo ""
echo "[2/3] Installing Python dependencies ..."
python3 -m pip install --break-system-packages mavsdk numpy opencv-python ultralytics cmake ninja empy jinja2 toml packaging pyyaml 2>&1 | tail -5
echo "  Python dependencies installed."

# ---- Step 3: Build PX4 SITL ----
echo ""
echo "[3/3] Building PX4 SITL ..."
if [ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    cd "$PX4_DIR"
    make px4_sitl_default -j$(nproc)
    echo "  PX4 SITL built successfully."
else
    echo "  PX4 SITL already built."
fi

# ---- Step 4: Gazebo (optional) ----
if [ -f "$GAZEBO_SETUP_SCRIPT" ]; then
    echo ""
    echo "Gazebo setup script found at $GAZEBO_SETUP_SCRIPT"
    echo "Run it separately to set up Gazebo simulation:"
    echo "  bash $GAZEBO_SETUP_SCRIPT"
fi

echo ""
echo "============================================"
echo " Setup complete!"
echo ""
echo "To run the simulation:"
echo "  1. Start PX4 SITL:"
echo "     bash $SCRIPT_DIR/run_sitl.sh"
echo ""
echo "  2. Run the mission:"
echo "     cd $PROJECT_DIR && python3 run_mission.py --sim"
echo ""
echo "  Or run both together:"
echo "     bash $SCRIPT_DIR/run_mission_sim.sh"
echo "============================================"
