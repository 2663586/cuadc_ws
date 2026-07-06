# setpoint 类型切换：位置 ↔ 速度

> 如何在 offboard 模式下安全切换 setpoint 类型，保证心跳不间断。

---

## 一、为什么需要切换

不同任务阶段适合不同的控制模式：

| 阶段 | 控制模式 | 原因 |
|------|---------|------|
| 起飞、降落 | 位置 | 需要精确到达固定点 |
| 巡航、搜索 | 位置 | 航线固定，位置控制简单可靠 |
| 视觉伺服对准 | 速度 | 根据视觉偏移量连续调整，比位置更平滑 |
| 避障 | 速度 | 避开障碍物需要即时方向响应，位置模式太慢 |

PX4 offboard 模式下，位置和速度走不同的控制流水线：

```
set_position_ned() → Position Controller → Velocity Controller → 混控 → 电机
set_velocity_ned() ───────────────────────── Velocity Controller → 混控 → 电机
```

速度指令跳过位置环，直接进入速度环，响应更快但需要外部位置边界保护。

---

## 二、核心安全约束

**PX4 offboard 超时 = 500ms。** 如果 500ms 内没收到任何有效 setpoint，PX4 自动退出 offboard 模式，切到 Hold。切换 setpoint 类型时，两次心跳之间的间隔必须 **< 500ms**。

当前心跳 20 Hz = 50ms 间隔，有 10 倍余量。

---

## 三、当前心跳的局限

```python
# interface.py — 当前心跳只支持一种 setpoint 类型
async def _heartbeat_loop(self):
    while True:
        await self.drone.offboard.set_position_ned(self._last_setpoint)
        await asyncio.sleep(interval)
```

硬编码了 `set_position_ned()`。如果直接把 `_last_setpoint` 换成 `VelocityNedYaw` 对象，方法签名不匹配，调用抛出 `TypeError`。

---

## 四、方案：心跳支持多类型 setpoint

把心跳改为 setpoint 类型感知。核心改动三处：

### 4.1 存储当前 setpoint 类型

```python
class PX4Interface:
    def __init__(self, ...):
        self._last_setpoint = PositionNedYaw(0, 0, 0, 0)
        self._setpoint_type = "position"  # "position" | "velocity"
```

### 4.2 分别提供更新接口

```python
    def update_position_setpoint(self, sp: PositionNedYaw):
        """切换到位置控制模式。"""
        self._last_setpoint = sp
        self._setpoint_type = "position"

    def update_velocity_setpoint(self, sp: VelocityNedYaw):
        """切换到速度控制模式。"""
        self._last_setpoint = sp
        self._setpoint_type = "velocity"
```

切换是纯内存操作（改引用 + 改字符串），不阻塞事件循环。

### 4.3 心跳根据当前类型调用对应方法

```python
    async def _heartbeat_loop(self):
        from config import OFFBOARD_HEARTBEAT_HZ
        interval = 1.0 / OFFBOARD_HEARTBEAT_HZ
        while True:
            if self._setpoint_type == "position":
                await self.drone.offboard.set_position_ned(self._last_setpoint)
            elif self._setpoint_type == "velocity":
                await self.drone.offboard.set_velocity_ned(self._last_setpoint)
            await asyncio.sleep(interval)
```

---

## 五、切换时序

```
心跳周期 N:   set_position_ned(pos)         ← 最后一帧位置指令
                    │
update_velocity_setpoint(vel)               ← 切换类型（改两个变量，< 1μs）
                    │
心跳周期 N+1: set_velocity_ned(vel)         ← 第一帧速度指令，间隔 < 50ms
```

**远远小于 500ms 超时窗口，PX4 不会退出 offboard。**

PX4 内部的速度环会以当前测量速度作为初始状态，然后追踪新目标——这个过程本身是连续的，不需要外部做插值。

---

## 六、从速度切回位置

反向切换同样简单：

```python
# 视觉伺服完成，切回航线巡航
sp = interface.field_to_ned(next_waypoint_x, next_waypoint_y, CRUISE_ALTITUDE_M)
interface.update_position_setpoint(sp)
```

心跳下一帧自动从 `set_velocity_ned()` 切到 `set_position_ned()`。PX4 会从当前测量位置出发，规划一条到目标位置的轨迹。

---

## 七、速度模式的安全保护

速度模式下，飞机可能累积漂移。必须同时施加位置边界：

```python
class VelocitySearchState(BaseState):
    """速度控制搜索，位置边界保护。"""

    async def execute(self, interface):
        # 发送速度指令
        vel = VelocityNedYaw(
            north_m_s=2.0,    # 2 m/s 向前搜索
            east_m_s=0.0,
            down_m_s=0.0,     # 保持高度
            yaw_deg=self.FIELD_YAW_DEG,
        )
        interface.update_velocity_setpoint(vel)

        # 位置边界检查
        pos = await interface.get_position_ned()
        if self._out_of_bounds(pos):
            # 越界：强制切回位置模式，停在边界
            sp = interface.field_to_ned(self._max_forward, 0, self._altitude)
            interface.update_position_setpoint(sp)
            return ExecutionResult(error="超出搜索边界")

        return ExecutionResult()
```

---

## 八、安全清单

| 风险 | 处理 |
|------|------|
| **切换时空窗 > 500ms** | 心跳 50ms 间隔，切类型只是一个 if 分支，无空窗 |
| **PX4 收到速度指令后立刻运动** | 初始速度设为 `(0,0,0,0)`，飞机悬停等待后续指令 |
| **速度模式累积漂移** | 定期读 `get_position_ned()` 校验，越界切回位置模式 |
| **GPS 丢失时速度模式不可靠** | `global_guard_check` 继续工作，gps_fix 不够时切 RTL |
| **两个状态同时改 setpoint 类型** | FSM 单线程，execute 顺序执行，不存在竞态 |
| **心跳 if 分支报错** | 每种 setpoint 类型对应不同的 MAVSDK 方法，必须 1:1 匹配；未来新增类型（如 attitude）只需加 elif |

---

## 九、与其他设计的关联

- 心跳机制设计：参见 [状态机设计.md](状态机设计.md) 第七章
- 栈式状态机抢占机制：参见 [状态机设计.md](状态机设计.md) 第三章
- PX4 offboard 模式详解：参见 [mavsdk/offboard.md](mavsdk/offboard.md)
