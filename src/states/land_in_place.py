"""
原地降落 —— 在当前水平位置直接触发 PX4 着陆，不返回原点。
"""

from .base_state import BaseState
from logger_manager import get_logger


class LandInPlaceState(BaseState):
    """原地降落：在当前水平位置直接着陆，等待 PX4 自动 disarm。"""

    def __init__(self, timeout_s: float = 60):
        super().__init__("LandInPlace", timeout_s)

    async def enter(self, interface):
        await super().enter(interface)
        alt = await interface.get_altitude()
        print(f"[降落] 原地着陆，当前高度 {alt:.1f} 米")
        get_logger().log_message("land", f"原地着陆，当前高度 {alt:.1f} 米")
        await interface.land()

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "降落超时"
            get_logger().log_message("land", "降落超时", "timeout")
            return True, None

        async for armed in interface.drone.telemetry.armed():
            if not armed:
                self.is_completed = True
                print("[降落] 着陆完成，已自动断开上锁")
                get_logger().log_message("land", "着陆完成，已自动断开上锁")
                return True, None
            break

        return False, None
