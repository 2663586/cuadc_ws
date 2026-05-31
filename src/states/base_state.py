"""Base class for all mission states."""

import time
from abc import ABC, abstractmethod
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from interface import PX4Interface


class BaseState(ABC):
    """Abstract base for all states."""

    def __init__(self, name: str, timeout_s: Optional[float] = None):
        self.name = name
        self.timeout_s = timeout_s  # None = no timeout
        self._enter_time: Optional[float] = None
        self.is_completed = False
        self.error: Optional[str] = None

    async def enter(self, interface: "PX4Interface"):
        """Called when entering the state."""
        self._enter_time = time.monotonic()
        self.is_completed = False
        self.error = None

    @abstractmethod
    async def execute(self, interface: "PX4Interface"):
        """
        Called every main-loop cycle.
        Returns (done: bool, next_state: Optional[BaseState]).
        """
        ...

    async def exit(self, interface: "PX4Interface"):
        """Called when exiting the state. Override for cleanup."""
        pass

    def elapsed(self) -> float:
        """Seconds since this state was entered."""
        if self._enter_time is None:
            return 0.0
        return time.monotonic() - self._enter_time

    def is_timed_out(self) -> bool:
        """Check whether the state has exceeded its time limit."""
        if self.timeout_s is None:
            return False
        return self.elapsed() > self.timeout_s
