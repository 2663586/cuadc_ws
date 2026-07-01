"""所有任务状态的基类。"""

import time
from abc import ABC, abstractmethod
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from interface import PX4Interface


class BaseState(ABC):
    """所有状态的抽象基类。"""

    def __init__(self, name: str, timeout_s: Optional[float] = None):
        self.name = name
        self.timeout_s = timeout_s  # None = 无超时限制
        self._enter_time: Optional[float] = None
        self.is_completed = False
        self.error: Optional[str] = None

    async def enter(self, interface: "PX4Interface"):
        """进入状态时调用。"""
        self._enter_time = time.monotonic()
        self.is_completed = False
        self.error = None

    @abstractmethod
    async def execute(self, interface: "PX4Interface"):
        """
        每个主循环周期调用。
        返回 (done: bool, next_state: Optional[BaseState])。
        """
        ...

    async def exit(self, interface: "PX4Interface"):
        """退出状态时调用。可重写以进行清理。"""
        pass

    def elapsed(self) -> float:
        """自进入此状态以来经过的秒数。"""
        if self._enter_time is None:
            return 0.0
        return time.monotonic() - self._enter_time

    def is_timed_out(self) -> bool:
        """检查状态是否已超过其时间限制。"""
        if self.timeout_s is None:
            return False
        return self.elapsed() > self.timeout_s
