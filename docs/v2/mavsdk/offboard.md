# offboard

> 文件来源：`mavsdk/offboard.py`（自动生成，来自 [MAVSDK-Python](https://github.com/mavlink/MAVSDK-Python)）
>
> 作用：Python 与 PX4 飞控之间的 offboard 模式通信接口，底层通过 gRPC → mavsdk_server → MAVLink 与飞控交互。

---

## 一、文件结构总览

共 1233 行，分三大类：

| 类别 | 数量 | 说明 |
|------|------|------|
| 设定值数据类型（Setpoint Types） | 7 个 | 纯数据容器 + gRPC 序列化 |
| 执行器控制类型 | 2 个 | 舵机/电机直接控制 |
| 结果/错误类型 | 2 个 | 指令返回值与异常 |
| 核心 `Offboard` 类 | 1 个 | 所有 offboard 操作方法 |

---

## 二、7 个设定值数据类型

每个类型结构完全一致，以最常用的 `PositionNedYaw` 为例：

```python
PositionNedYaw(north_m, east_m, down_m, yaw_deg)
```

**每个类型共有的方法：**

| 方法 | 作用 |
|------|------|
| `__init__` | 构造，传入各字段值 |
| `__eq__` | 比较两个对象是否相等 |
| `__str__` | 人类可读字符串，如 `PositionNedYaw: [north_m: 5.0, east_m: 0.0, ...]` |
| `translate_to_rpc()` | Python 对象 → protobuf，发往飞控前调用 |
| `translate_from_rpc()` (static) | protobuf → Python 对象，从飞控接收数据时调用 |

**七个类型对照表：**

| 类型 | 控制物理量 | 坐标系统 | 本项目使用情况 |
|------|-----------|----------|:---:|
| `PositionNedYaw` | 位置 + 航向 | NED (北-东-地) | ✅ 核心使用 |
| `PositionGlobalYaw` | 经纬度 + 高度 + 航向 | GPS 全球坐标 | ❌ |
| `VelocityNedYaw` | 速度 + 航向 | NED | ❌ |
| `VelocityBodyYawspeed` | 速度 + 航向角速率 | 机体坐标系 | ❌ |
| `AccelerationNed` | 加速度 | NED | ❌ |
| `Attitude` | 姿态角 (roll/pitch/yaw) + 油门 | NED 参考系 | ❌ |
| `AttitudeRate` | 姿态角速率 + 油门 | 机体坐标系 | ❌ |

### PositionGlobalYaw 内置枚举 `AltitudeType`

```python
class AltitudeType(Enum):
    REL_HOME = 0   # 相对于家点（Home）高度
    AMSL = 1       # 海拔高度
    AGL = 2        # 离地高度
```

---

## 三、执行器控制类型

| 类型 | 作用 |
|------|------|
| `ActuatorControlGroup` | 一组 8 个控制量，每个归一化到 -1..+1，通过混控器映射到实际输出 |
| `ActuatorControl` | 最多 2 组（共 16 个控制量）。前 8 个映射到控制组 0，后 8 个到控制组 1 |

> 注：本项目目前通过 `action.set_actuator()` 控制舵机，未使用 offboard 执行器控制。

---

## 四、结果与错误类型

### OffboardResult

每个 offboard 指令的返回值结构：

| 字段 | 类型 | 含义 |
|------|------|------|
| `result` | `OffboardResult.Result` 枚举 | 状态码 |
| `result_str` | `str` | 人类可读的英文描述 |

### Result 枚举值

| 枚举值 | 整数 | 含义 |
|--------|------|------|
| `UNKNOWN` | 0 | 未知结果 |
| `SUCCESS` | 1 | ✅ 指令成功 |
| `NO_SYSTEM` | 2 | 没有连接的系统 |
| `CONNECTION_ERROR` | 3 | 连接错误 |
| `BUSY` | 4 | 飞控忙 |
| `COMMAND_DENIED` | 5 | 指令被拒绝 |
| `TIMEOUT` | 6 | 请求超时 |
| `NO_SETPOINT_SET` | 7 | **未先设置 setpoint 就 start** |
| `FAILED` | 8 | 通用失败 |

> 第 7 号 `NO_SETPOINT_SET` 是常见陷阱：必须在调用 `start()` 之前至少发送一次 `set_position_ned()`，否则 PX4 拒绝进入 offboard 模式。

### OffboardError

当 `Result != SUCCESS` 时抛出的异常类，包含 `_result`、`_origin`（调用方法名）、`_params`（传入参数）。

---

## 五、核心类 `Offboard(AsyncBase)`

项目中通过 `drone.offboard` 访问该类的实例。内部通过 gRPC stub 与 `mavsdk_server` 通信：

```python
def _setup_stub(self, channel):
    self._stub = offboard_pb2_grpc.OffboardServiceStub(channel)
```

### 生命周期方法

| 方法 | 作用 | 项目中调用位置 |
|------|------|---------------|
| `start()` | 切换 PX4 到 offboard 模式 | `interface.py: arm_and_offboard()` |
| `stop()` | 退出 offboard 模式，飞控切到 Hold | `interface.py: disarm()` |
| `is_active()` | 查询 offboard 是否活跃 | 未使用 |

### Setpoint 发送方法（按控制层级从低到高排列）

| 方法 | 控制层级 | 前馈项 |
|------|----------|--------|
| `set_actuator_control()` | 舵机/电机直接控制 | — |
| `set_attitude_rate()` | 角速率 + 油门 | — |
| `set_attitude()` | 姿态角 + 油门 | — |
| `set_acceleration_ned()` | NED 加速度 | — |
| `set_velocity_body()` | 机体速度 + 角速率 | — |
| `set_velocity_ned()` | NED 速度 + 航向 | — |
| `set_position_ned()` | **NED 位置 + 航向** | — |
| `set_position_global()` | GPS 位置 + 航向 | — |
| `set_position_velocity_ned()` | 位置 + 速度 | 速度前馈 |
| `set_position_velocity_acceleration_ned()` | 位置 + 速度 + 加速度 | 速度 + 加速度前馈 |

层级越高（带前馈），PX4 跟踪轨迹越平滑。本项目使用 `set_position_ned()` — 最常用的位置控制模式。

### 每个 set 方法的内部流程

```python
async def set_position_ned(self, position_ned_yaw):
    request = offboard_pb2.SetPositionNedRequest()       # 1. 创建 protobuf 请求
    position_ned_yaw.translate_to_rpc(request.xxx)        # 2. Python 对象 → protobuf
    response = await self._stub.SetPositionNed(request)   # 3. gRPC 发往 mavsdk_server
    result = self._extract_result(response)                # 4. 解包结果枚举
    if result.result != OffboardResult.Result.SUCCESS:     # 5. 失败则抛 OffboardError
        raise OffboardError(result, "set_position_ned()", position_ned_yaw)
```

**关键点**：`set_position_ned()` 不是直接发 MAVLink 给飞控，数据链路为：

```
Python (你的代码)
    │ gRPC
    ▼
mavsdk_server (C++ 二进制)
    │ MAVLink (UDP)
    ▼
PX4 飞控 (SITL 或真机)
```

每次调用都是一次完整的 gRPC round-trip。

---

## 六、与项目代码的对应关系

```
你的 interface.py                    offboard.py 中的对应
───────────────────────────────      ──────────────────────
drone.offboard.start()           →   Offboard.start()
drone.offboard.stop()            →   Offboard.stop()
drone.offboard.set_position_ned() →  Offboard.set_position_ned()
PositionNedYaw(north, east,      →   PositionNedYaw 类
                down, yaw)
```

`_heartbeat_loop` 以 20 Hz 频率循环调用 `set_position_ned()`，即每 50ms 走一次上述 gRPC 流程。PX4 要求 offboard setpoint 最低 2 Hz，20 Hz 提供 10 倍余量。

---

## 七、常见问题

### Q: 为什么 `start()` 前必须先发一个 setpoint？

A: PX4 的安全机制。如果没有先设置目标位置就切换到 offboard，飞控不知道该去哪里。结果码 `NO_SETPOINT_SET (7)` 就是这个场景。项目中 `arm_and_offboard()` 在 `start()` 前先发送 `PositionNedYaw(0,0,0,0)` 作为初始锁定值。

### Q: 项目中为什么选择 `set_position_ned` 而不是其他模式？

A: 任务要求按预定航点飞行（投掷区、侦察区），NED 位置控制最直接。速度模式 (`set_velocity_ned`) 更适合遥操作；姿态模式 (`set_attitude`) 适合特技飞行；前馈模式 (`set_position_velocity_ned`) 适合需要平滑轨迹的场景。
