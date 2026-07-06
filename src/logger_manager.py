"""
黑匣子日志记录器 —— 记录任务数据用于事后分析和调试。

CSV 格式，四列：t（相对时间）, type（类型标签）, content（内容）, result（结果）。
提供模块级 init_logger() / get_logger() 单例接口。
"""

import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

_logger_instance: Optional["LoggerManager"] = None


def init_logger(log_dir: str = "./logs") -> "LoggerManager":
    """初始化全局日志记录器。必须在程序启动时调用一次。"""
    global _logger_instance
    _logger_instance = LoggerManager(log_dir)
    return _logger_instance


def get_logger() -> "LoggerManager":
    """获取全局日志记录器实例。需先调用 init_logger()。"""
    if _logger_instance is None:
        raise RuntimeError("日志记录器未初始化，请先调用 init_logger()")
    return _logger_instance


class LoggerManager:
    """任务黑匣子日志记录器。"""

    def __init__(self, log_dir: str = "./logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        self.log_file = self.log_dir / \
            f"mission_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self._start_time = time.monotonic()
        self._csv_file = open(str(self.log_file), "a", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["t", "type", "content", "result"])
        self._csv_file.flush()
        print(f"[日志] 记录到 {self.log_file}")

    def _now(self) -> float:
        """返回相对于启动时间的秒数。"""
        return round(time.monotonic() - self._start_time, 3)

    # ------------------------------------------------------------------
    # 通用消息日志 —— 对应所有 print() 输出
    # ------------------------------------------------------------------

    def log_message(self, msg_type: str, content: str, result: str = "ok"):
        """
        记录一条带类型的消息。

        参数:
            msg_type: 类型标签（英文），如 "info"、"error"、"takeoff" 等
            content:  消息正文
            result:   结果标记，默认 "ok"；失败用 "fail"，超时用 "timeout"
        """
        self._csv.writerow([self._now(), msg_type, content, result])
        self._csv_file.flush()

    # ------------------------------------------------------------------
    # 结构化事件
    # ------------------------------------------------------------------

    def log_state_transition(self, from_state: str, to_state: str):
        self.log_message(
            "state_transition",
            json.dumps({"from": from_state, "to": to_state}, ensure_ascii=False),
        )

    def log_telemetry(self, health):
        """记录遥测快照。health 为 HealthStatus 实例。"""
        self.log_message(
            "telemetry",
            json.dumps({
                "connected": health.is_connected,
                "armed": health.is_armed,
                "offboard": health.is_offboard,
                "gps_ok": health.is_global_position_ok,
                "home_ok": health.is_home_position_ok,
                "battery_pct": health.battery_pct,
                "gps_fix": health.gps_fix_type,
            }, ensure_ascii=False),
        )

    def log_vision_result(self, state: str, num_cylinders: int):
        self.log_message(
            "vision",
            json.dumps({"state": state, "cylinders_detected": num_cylinders},
                       ensure_ascii=False),
        )

    def log_drop(self, bottle_index: int, position: Optional[tuple] = None):
        data = {"bottle": bottle_index}
        if position is not None:
            data["ned_offset"] = position
        self.log_message("drop", json.dumps(data, ensure_ascii=False))

    def log_recon(self, waypoint_index: int):
        self.log_message(
            "recon",
            json.dumps({"waypoint": waypoint_index}, ensure_ascii=False),
        )

    def log_event(self, event: str, **kwargs):
        """通用事件日志记录器。"""
        data = {"event": event}
        data.update(kwargs)
        self.log_message("event", json.dumps(data, ensure_ascii=False))
