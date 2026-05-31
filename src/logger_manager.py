"""
Black-box logger — records mission data for post-flight analysis and debugging.

Logs state transitions, telemetry snapshots, vision results, and drop events
to a timestamped file under logs/.
"""

import json
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from interface import HealthStatus


class LoggerManager:
    """Mission black-box logger."""

    def __init__(self, log_dir: str = "./logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        self.log_file = self.log_dir / \
            f"mission_{datetime.now():%Y%m%d_%H%M%S}.log"
        self._start_time = time.monotonic()
        print(f"[Logger] Recording to {self.log_file}")

    def _write(self, entry: dict):
        entry["t"] = time.monotonic() - self._start_time
        with open(self.log_file, "a") as f:
            f.write(json.dumps(entry) + "\n")

    def log_state_transition(self, from_state: str, to_state: str):
        self._write({
            "type": "state_transition",
            "from": from_state,
            "to": to_state,
        })

    def log_telemetry(self, health: HealthStatus):
        self._write({
            "type": "telemetry",
            "connected": health.is_connected,
            "armed": health.is_armed,
            "offboard": health.is_offboard,
            "gps_ok": health.is_global_position_ok,
            "home_ok": health.is_home_position_ok,
            "battery_pct": health.battery_pct,
            "gps_fix": health.gps_fix_type,
        })

    def log_vision_result(self, state: str, num_cylinders: int):
        self._write({
            "type": "vision",
            "state": state,
            "cylinders_detected": num_cylinders,
        })

    def log_drop(self, bottle_index: int, position: Optional[tuple] = None):
        entry = {
            "type": "drop",
            "bottle": bottle_index,
        }
        if position is not None:
            entry["ned_offset"] = position
        self._write(entry)

    def log_recon(self, waypoint_index: int):
        self._write({
            "type": "recon",
            "waypoint": waypoint_index,
        })

    def log_event(self, event: str, **kwargs):
        """Generic event logger."""
        entry = {"type": "event", "event": event}
        entry.update(kwargs)
        self._write(entry)
