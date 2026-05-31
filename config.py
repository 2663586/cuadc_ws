"""
Global parameters for the CUADC 2026 mission.

All state modules import from here — no hardcoded values.
Tune these before competition based on field measurements.
"""

# ---------------------------------------------------------------------------
# Field orientation — auto-detected at arm time.
# Place the aircraft on the takeoff pad facing the field forward direction;
# the program reads current heading at arm and overwrites this value.
# ---------------------------------------------------------------------------
FIELD_YAW_DEG = 0.0

# ---------------------------------------------------------------------------
# Flight parameters
# ---------------------------------------------------------------------------
CRUISE_ALTITUDE_M = 7.0          # safe cruise altitude after takeoff
DROP_ZONE_DISTANCE_M = 30.0      # distance from takeoff to drop zone
RECON_ZONE_DISTANCE_M = 55.0     # distance from takeoff to recon zone
DROP_ALIGN_ALTITUDE_M = 2.5      # altitude above cylinder for fine alignment
RECON_ALTITUDE_M = 2.5           # altitude for recon scanning
LAND_START_ALTITUDE_M = 5.0      # altitude when starting landing sequence
LAND_SAFE_ALTITUDE_M = 2.0       # below this altitude, use slow descent
TRANSIT_SPEED_MPS = 5.0          # cruise speed between zones

# ---------------------------------------------------------------------------
# Control parameters
# ---------------------------------------------------------------------------
OFFBOARD_HEARTBEAT_HZ = 20       # offboard setpoint send rate (must be >= 2)
FSM_LOOP_HZ = 20                 # state machine main loop rate
ALIGN_THRESHOLD_M = 0.05         # visual servoing alignment threshold
SEARCH_TIMEOUT_S = 3.0           # target re-acquisition timeout
VISUAL_SERVO_KP = 0.5            # visual servo P-controller gain
LAND_DESCEND_RATE_MPS = 0.3      # normal descent rate during landing
DROP_ALTITUDE_M = 5.0            # altitude during drop phase coarse approach

# ---------------------------------------------------------------------------
# Vision parameters
# ---------------------------------------------------------------------------
YOLO_CONFIDENCE_THRESHOLD = 0.5  # YOLO detection confidence threshold
RECON_CONFIDENCE_THRESHOLD = 0.7 # recon classification confidence threshold
RECON_SCAN_STEP_M = 1.5          # recon zone scan line spacing

# ---------------------------------------------------------------------------
# Safety parameters
# ---------------------------------------------------------------------------
BATTERY_LOW_THRESHOLD_PCT = 20.0 # trigger RTL below this battery percentage
GPS_FIX_MIN = 3                  # minimum GPS fix type (3 = 3D fix)
GLOBAL_GUARD_INTERVAL_S = 0.05   # health check interval (match FSM loop rate)
