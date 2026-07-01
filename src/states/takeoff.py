"""起飞状态 —— 爬升至目标高度。"""

from .base_state import BaseState


class TakeoffState(BaseState):
    """上锁 + offboard 后爬升至目标高度。"""

    def __init__(self, target_alt: float = 7.0, timeout_s: float = 30):
        super().__init__("Takeoff", timeout_s)
        self.target_alt = target_alt

    async def enter(self, interface):
        await super().enter(interface)
        # 设定起飞高度并指令起飞
        sp = interface.field_to_ned(0.0, 0.0, self.target_alt)
        interface.update_setpoint(sp)
        await interface.takeoff(self.target_alt)

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "起飞超时"
            return True, None

        alt = await interface.get_altitude()
        if alt >= self.target_alt * 0.9:
            self.is_completed = True
            return True, None
        return False, None
