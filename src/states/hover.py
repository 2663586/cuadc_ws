"""悬停状态 —— 在固定时长内保持位置。"""

from .base_state import BaseState, ExecutionResult


class HoverState(BaseState):
    """在指定时间内保持位置。心跳任务维持设定值。"""

    def __init__(self, hover_time: float = 1.0):
        super().__init__("Hover", timeout_s=hover_time + 5)
        self.hover_time = hover_time

    async def execute(self, interface):
        if self.elapsed() >= self.hover_time:
            self.is_completed = True
            return ExecutionResult(done=True)
        return ExecutionResult()
