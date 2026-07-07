"""巡航状态 —— 以巡航速度飞往场地 NED 坐标航点。"""

from typing import Optional

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult


class TransitState(BaseState):
    """
    以巡航速度飞往目标场地 NED 坐标。

    参数 north, east, up 均在"场地 NED"坐标系中：
      - north = 沿场地前方方向（米）
      - east  = 沿场地右方方向（米）
      - up    = 飞行高度（米，向上为正）

    设定值在 enter() 中发布一次，心跳任务维持该设定值。
    完成判定基于预计时间 + 高度接近度。
    """

    def __init__(self, north: float, east: float, up: float,
                 speed: float = 5.0, timeout_s: float = 60):
        super().__init__("Transit", timeout_s)
        self.target_north, self.target_east, self.target_up = north, east, up
        self.speed = speed

    async def enter(self, interface):
        await super().enter(interface)
        target_sp = interface.field_to_ned(self.target_north, self.target_east,
                                            self.target_up)
        interface.update_setpoint(target_sp)
        print(f"[巡航] -> 场地 NED({self.target_north:.1f}, "
              f"{self.target_east:.1f}, {self.target_up:.1f}) "
              f"@ {self.speed:.1f} 米/秒")

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "巡航超时"
            return ExecutionResult(done=True)

        alt = await interface.get_altitude()
        # 根据距离估算行进时间
        dist = (self.target_north ** 2 + self.target_east ** 2) ** 0.5
        est_time = dist / self.speed if self.speed > 0 else 10

        if self.elapsed() > est_time and abs(alt - self.target_up) < 0.5:
            self.is_completed = True
            return ExecutionResult(done=True)
        return ExecutionResult()
