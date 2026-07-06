"""
降落 —— 飞回初始点上空，触发 PX4 自动着陆，等待完成。
"""

from .base_state import BaseState
from logger_manager import get_logger


class PrecisionLandState(BaseState):
    """降落：飞回原点，触发 land，等待 PX4 着陆并自动 disarm。"""

    def __init__(self, timeout_s: float = 60):
        super().__init__("Land", timeout_s)

    async def enter(self, interface):
        await super().enter(interface)
        # 先回到原点上方
        alt = await interface.get_altitude()
        sp = interface.field_to_ned(0.0, 0.0, alt)
        interface.update_setpoint(sp)
        print(f"[降落] 返回原点上方 {alt:.1f} 米，触发着陆")
        get_logger().log_message(
            "land", f"返回原点上方 {alt:.1f} 米，触发着陆")
        # 触发 PX4 自动降落（切到 Land 模式，着陆后自动 disarm）
        await interface.land()

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "降落超时"
            get_logger().log_message("land", "降落超时", "timeout")
            return True, None

        # 检测着陆完成：PX4 着陆后自动 disarm，armed 变为 False
        async for armed in interface.drone.telemetry.armed():
            if not armed:
                self.is_completed = True
                print("[降落] 着陆完成，已自动断开上锁")
                get_logger().log_message("land", "着陆完成，已自动断开上锁")
                return True, None
            break

        return False, None


# ============================================================================
# 原精准降落逻辑（视觉 + GPS 回退，暂时注释保留备用）
# ============================================================================
#
# """
# 精准降落 —— 双策略。
# ...
# """
