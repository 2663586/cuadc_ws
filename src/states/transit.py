"""巡航状态 —— 以巡航速度飞往场地坐标航点。"""

from typing import Optional

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult


class TransitState(BaseState):
    """
    以巡航速度飞往目标场地坐标（前、右、高）。

    设定值在 enter() 中发布一次，心跳任务维持该设定值。
    完成判定基于预计时间 + 高度接近度。
    """

    def __init__(self, x: float, y: float, z: float,
                 speed: float = 5.0, timeout_s: float = 60):
        super().__init__("Transit", timeout_s)
        self.target_x, self.target_y, self.target_z = x, y, z
        self.speed = speed

    async def enter(self, interface):
        await super().enter(interface)
        target_sp = interface.field_to_ned(self.target_x, self.target_y,
                                            self.target_z)
        interface.update_setpoint(target_sp)
        print(f"[巡航] -> 场地({self.target_x:.1f}, {self.target_y:.1f}, "
              f"{self.target_z:.1f}) @ {self.speed:.1f} 米/秒")

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "巡航超时"
            return ExecutionResult(done=True)

        alt = await interface.get_altitude()
        # 根据距离估算行进时间
        dist = (self.target_x ** 2 + self.target_y ** 2) ** 0.5
        est_time = dist / self.speed if self.speed > 0 else 10

        if self.elapsed() > est_time and abs(alt - self.target_z) < 0.5:
            self.is_completed = True
            return ExecutionResult(done=True)
        return ExecutionResult()
