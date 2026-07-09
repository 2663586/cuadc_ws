# setpoint 类型切换：位置 ↔ 速度 ↔ 位置+速度

> 如何在 offboard 模式下安全切换 setpoint 类型，保证心跳不间断。

---

## 一、为什么需要切换

不同任务阶段适合不同的控制模式：

| 阶段 | 控制模式 | 原因 |
|------|---------|------|
| 起飞、降落 | 位置 | 需要精确到达固定点 |
| 巡航、搜索 | **位置+速度** | 航线固定但需限速以保证图像质量，速度前馈控制实际飞行速度 |
| 视觉伺服对准 | 速度 | 根据视觉偏移量连续调整，比位置更平滑 |
| 悬停保持 | 位置 | 零速保持，位置控制器天然适合 |
| 避障 | 速度 | 避开障碍物需要即时方向响应，位置模式太慢 |

PX4 offboard 模式下，三种控制走不同的流水线：

```
set_position_ned() ─────────────→ Position Controller → Velocity Controller → 混控 → 电机
set_velocity_ned() ───────────────────────────────────── Velocity Controller → 混控 → 电机
set_position_velocity_ned() ────→ Position Controller → Velocity Controller → 混控 → 电机
                                       ↑ 位置输出 + 速度前馈叠加
```

- **纯位置**：位置控制器根据位置误差计算所需速度 → 速度环执行
- **纯速度**：跳过位置环，直接设目标速度，响应快但无位置边界
- **位置+速度**：位置控制器输出 + 速度前馈 = 总速度指令。位置误差小时速度前馈主导，误差大时两者叠加

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

## 四、方案：心跳支持三种 setpoint 类型

把心跳改为 setpoint 类型感知。核心改动三处：

### 4.1 存储当前 setpoint 类型

```python
class PX4Interface:
    def __init__(self, ...):
        self._last_setpoint = PositionNedYaw(0, 0, 0, 0)
        self._last_velocity = VelocityNedYaw(0, 0, 0, 0)
        self._setpoint_type = "position"  # "position" | "velocity" | "position_velocity"
```

需要分开存储 `_last_setpoint`（位置，`PositionNedYaw`）和 `_last_velocity`（速度，`VelocityNedYaw`），因为 `set_position_velocity_ned` 需要同时传入两者，但 `set_position_ned` 和 `set_velocity_ned` 各只需要一个。

### 4.2 分别提供更新接口

```python
    def update_position_setpoint(self, sp: PositionNedYaw):
        """切换到纯位置控制模式。"""
        self._last_setpoint = sp
        self._setpoint_type = "position"

    def update_velocity_setpoint(self, sp: VelocityNedYaw):
        """切换到纯速度控制模式。"""
        self._last_velocity = sp
        self._setpoint_type = "velocity"

    def update_position_velocity_setpoint(self, pos: PositionNedYaw, vel: VelocityNedYaw):
        """切换到位置+速度前馈模式 —— 搜索航线限速使用。"""
        self._last_setpoint = pos
        self._last_velocity = vel
        self._setpoint_type = "position_velocity"

    def clear_velocity(self):
        """
        紧急清除速度前馈 —— suspend 时调用。
        
        将 _last_velocity 归零并切回位置模式，
        确保心跳退化为纯位置保持，避免飞机漂移。
        """
        self._last_velocity = VelocityNedYaw(0, 0, 0, self.FIELD_YAW_DEG)
        self._setpoint_type = "position"
```

切换是纯内存操作（改引用 + 改字符串），不阻塞事件循环。

### 4.3 心跳根据当前类型调用对应方法

```python
    async def _heartbeat_loop(self):
        from config import OFFBOARD_HEARTBEAT_HZ
        interval = 1.0 / OFFBOARD_HEARTBEAT_HZ
        while self._heartbeat_running:
            if self._setpoint_type == "position":
                await self.drone.offboard.set_position_ned(self._last_setpoint)
            elif self._setpoint_type == "velocity":
                await self.drone.offboard.set_velocity_ned(self._last_velocity)
            elif self._setpoint_type == "position_velocity":
                await self.drone.offboard.set_position_velocity_ned(
                    self._last_setpoint, self._last_velocity)
            await asyncio.sleep(interval)
```

**注意**：`set_position_velocity_ned` 需要 MAVSDK >= 2.0。MAVSDK-Python 已确认支持此方法。

---

## 五、切换时序

### 5.1 位置 → 位置+速度

```
心跳周期 N:   set_position_ned(pos)              ← 最后一帧纯位置指令
                    │
update_position_velocity_setpoint(pos, vel)       ← 切换类型（改三个变量，< 1μs）
                    │
心跳周期 N+1: set_position_velocity_ned(pos, vel) ← 第一帧位置+速度指令，间隔 < 50ms
```

### 5.2 位置+速度 → 位置（suspend / 状态退出）

```
心跳周期 N:   set_position_velocity_ned(pos, vel) ← 最后一帧带速度指令
                    │
clear_velocity()                                   ← velocity 归零，切回 position
                    │
心跳周期 N+1: set_position_ned(pos)                ← 退化为纯位置保持，飞机悬停
```

### 5.3 位置 → 速度（视觉伺服）

```
心跳周期 N:   set_position_ned(pos)               ← 最后一帧位置指令
                    │
update_velocity_setpoint(vel)                      ← 切换类型
                    │
心跳周期 N+1: set_velocity_ned(vel)                ← 第一帧速度指令
```

**所有切换间隔 < 50ms，远远小于 500ms offboard 超时窗口，PX4 不会退出 offboard。**

PX4 内部的速度环会以当前测量速度作为初始状态追踪新目标——这个过程是连续的，不需要外部做插值。

---

## 六、⚠️ 陷阱 1：Suspend 时的速度残留

### 问题场景

SearchState 使用 `set_position_velocity_ned(pos, vel)` 飞行矩形航线。当视觉检测到目标、触发栈式抢占时：

```
SearchState 检测到目标
  → return ExecutionResult(interrupt=AlignState)
    → main_fsm 调用 SearchState.suspend()
      → 此时 _last_velocity 仍指向航线方向（例如向北 3m/s）
        → AlignState.enter() 调用 update_position_setpoint() 之前
          → 心跳用残留 velocity 多发 1~2 个周期
            → 飞机继续向北漂移！
```

### 根本原因

`suspend()` 到 `AlignState.enter()` 之间存在时间窗口：
- FSM 主循环在 suspend 后立即 `_stack.append(AlignState)`
- 但 `AlignState.enter()` 要等到**下一个循环迭代**才执行
- 这期间心跳照常发送，用的是旧 velocity

### 修复

`SearchState.suspend()` 中显式清除速度：

```python
# SearchState
async def suspend(self, interface):
    await super().suspend(interface)
    interface.clear_velocity()  # velocity 归零 → 心跳退化为纯位置保持，飞机悬停
```

调用 `clear_velocity()` 后：
- `_last_velocity` 变为 `(0,0,0,0)`
- `_setpoint_type` 切回 `"position"`
- 心跳下一帧发送 `set_position_ned(last_pos)` → 飞机立即悬停
- `AlignState.enter()` 再调用 `update_position_setpoint(new_pos)` → 飞机飞向对准位置

### 全状态切换时序

```
SearchState 执行    → heartbeat 发 set_position_velocity_ned(ramped_pos, vel)  巡航搜索
SearchState.suspend → clear_velocity() → heartbeat 发 set_position_ned(last_pos)  ✋ 悬停
AlignState.enter    → update_position_setpoint(new_pos) → heartbeat 发 set_position_ned(new_pos)  飞向目标
AlignState 执行     → update_velocity_setpoint(servo_vel) → heartbeat 发 set_velocity_ned(vel)  视觉伺服
AlignState.exit     → (无需操作，FSM 自动 pop)
SearchState.resume  → update_position_velocity_setpoint(ramped_pos, vel) → 切回位置+速度  继续搜索
```

---

## 七、⚠️ 陷阱 2：位置大阶跃 + 速度前馈叠加超速

### 问题场景

如果直接把 position setpoint 设为最终航点坐标：

```
当前位置 ●──────────────────────────● 目标航点 (距离 6m)
                     ↓
set_position_velocity_ned(goal_pos, vel=3m/s →)
                     ↓
PX4 位置控制器:  误差 6m → 全力输出 ~5m/s
PX4 速度前馈:    + 3m/s
实际速度:        ~8m/s !!  远超预期的 SEARCH_SPEED_MPS
```

### 根本原因

`set_position_velocity_ned` 的速度分量是**前馈项**，叠加在位置控制器的输出上，不是替代。MAVLink `SET_POSITION_TARGET_LOCAL_NED` 的 type_mask 中位置和速度 bit 同时为 0（均激活），PX4 会将两者相加。

- 位置误差大 → 位置控制器输出大
- 速度前馈固定 = SEARCH_SPEED_MPS
- 总输出 = 位置控制器 + 前馈 > SEARCH_SPEED_MPS

### 修复：position setpoint 增量斜坡

每个 FSM 周期只把 position setpoint 推进一小步，保持位置误差始终很小：

```
周期 0:  pos_setpoint = current_pos                    误差 ≈ 0
周期 1:  pos_setpoint += dir × (speed/FSM_LOOP_HZ)     误差 ≈ 0.15m
周期 2:  pos_setpoint += dir × (speed/FSM_LOOP_HZ)     误差 ≈ 0.15m
...
```

位置误差始终 ≈ `speed / FSM_LOOP_HZ = 3.0 / 20 = 0.15m`，位置控制器输出 ≈ 0。速度前馈成为实际速度的主导项 → 实际飞行速度 ≈ `SEARCH_SPEED_MPS`。

### 完整实现

```python
async def _fly_to_target(self, interface, wp_idx):
    x, y, z, speed = self._rect_waypoints[wp_idx]
    target_ned = interface.field_to_ned(x, y, z)

    pos = await interface.get_position_ned()
    dn = target_ned.north_m - pos.north_m
    de = target_ned.east_m - pos.east_m
    dist = math.hypot(dn, de)

    # 到达判据：相对误差
    seg_length = self._segment_lengths[wp_idx]
    if dist < seg_length * ARRIVAL_REL_THRESHOLD:
        # 已到达 → position snap 到航点，velocity 归零
        interface.update_position_velocity_setpoint(
            target_ned,
            VelocityNedYaw(0, 0, 0, target_ned.yaw_deg))
        return True  # arrived

    # 增量斜坡：本周期只推进 step 距离
    step = speed / FSM_LOOP_HZ
    frac = step / dist if dist > 0 else 1.0
    if frac > 1.0:
        frac = 1.0
    intermediate = PositionNedYaw(
        pos.north_m + dn * frac,
        pos.east_m + de * frac,
        target_ned.down_m,          # z 直接 snap 到巡航高度
        target_ned.yaw_deg,
    )
    # 速度前馈：方向指向目标，大小 = SEARCH_SPEED_MPS
    vel = VelocityNedYaw(
        (dn / dist) * speed if dist > 0 else 0,
        (de / dist) * speed if dist > 0 else 0,
        0,                           # 保持高度
        target_ned.yaw_deg,
    )
    interface.update_position_velocity_setpoint(intermediate, vel)
    return False  # still en route
```

**关键点**：
- `intermediate` 只从当前位置推进 `step` 距离 → 位置误差始终很小
- `vel` 始终指向目标方向，大小 = `SEARCH_SPEED_MPS` → 主导实际飞行速度
- z 坐标直接 snap 到巡航高度 → 高度控制不受斜坡影响
- 到达时 velocity 归零 → 飞机在航点悬停，等待视觉检测

---

## 八、安全清单

| 风险 | 处理 |
|------|------|
| **切换时空窗 > 500ms** | 心跳 50ms 间隔，切类型只是改变量+if分支，无空窗 |
| **PX4 收到速度指令后立刻运动** | 初始速度设为 `(0,0,0,0)`，飞机悬停等待后续指令 |
| **Suspend 时速度残留 → 漂移** | `SearchState.suspend()` 调 `clear_velocity()`，velocity 归零切回位置模式 |
| **位置大阶跃 + 速度前馈 → 超速** | position setpoint 走增量斜坡，每周期只推进 `speed/FSM_LOOP_HZ`，位置误差始终 < 0.15m |
| **速度模式累积漂移** | 定期读 `get_position_ned()` 校验，越界切回位置模式 |
| **GPS 丢失时速度模式不可靠** | `global_guard_check` 继续工作，gps_fix 不够时切 RTL |
| **两个状态同时改 setpoint 类型** | FSM 单线程，execute 顺序执行，不存在竞态 |
| **心跳 if 分支报错** | 三种 setpoint 类型对应不同 MAVSDK 方法，必须严格匹配；新增类型只需加 elif |
| **`set_position_velocity_ned` 不可用** | 已确认 MAVSDK-Python 支持此方法（`Offboard` 类中可见） |

---

## 九、与其他设计的关联

- 心跳机制设计：参见 [状态机设计.md](状态机设计.md) 第七章
- 栈式状态机抢占机制：参见 [状态机设计.md](状态机设计.md) 第三章
- PX4 offboard 模式详解：参见 [mavsdk/offboard.md](mavsdk/offboard.md)
