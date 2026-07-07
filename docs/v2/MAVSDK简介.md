
## MAVSDK Python 概述

MAVSDK 是一套跨平台的 MAVLink 库，提供高层次 API 用于与 PX4/ArduPilot 飞控通信。Python 封装基于 asyncio，通过 gRPC 与 MAVSDK C++ 后端（`mavsdk_server`）通信，运行在机载电脑上。

**版本信息**：本项目使用 MAVSDK Python v3.15.3 + PX4 v1.16.1。

### 架构原理

```
机载电脑 (Raspberry Pi 5 / Jetson)
┌─────────────────────────────────────┐
│ Python 程序                          │
│  ┌─────────────────────────────┐    │
│  │ MAVSDK Python (asyncio)     │    │
│  │  ├─ Action 插件        │    │
│  │  ├─ Offboard 插件      │    │
│  │  ├─ Telemetry 插件     │    │
│  │  └─ Param 插件         │    │
│  └──────────┬──────────────────┘    │
│             │ gRPC                   │
│  ┌──────────▼──────────────────┐    │
│  │ mavsdk_server (C++ backend) │    │
│  └──────────┬──────────────────┘    │
└─────────────┼───────────────────────┘
              │ MAVLink (UDP/Serial)
┌─────────────▼───────────────────────┐
│ PX4 飞控                             │
└─────────────────────────────────────┘
```

连接方式：`System.connect("udp://:14540")` — 这是 PX4 默认的 offboard 端口。

### 核心插件及其用途

#### 1. Action 插件 — 基本飞行动作

提供最常用的飞行指令，不需要进入 offboard 模式。飞控利用自身的 position controller 完成目标。

```python
from mavsdk import System
import asyncio

async def basic_flight():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # 等待连接和 GPS 就绪
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    # Arm and takeoff
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(7.0)  # 设置起飞高度 7m
    await drone.action.takeoff()
    await asyncio.sleep(8)

    # 飞行到 GPS 坐标
    # goto_location(lat, lon, altitude_amsl, yaw)
    await drone.action.goto_location(30.0, 0.0, 500.0, 0.0)
    await asyncio.sleep(15)

    # 悬停
    await drone.action.hold()
    await asyncio.sleep(2)

    # 返航
    await drone.action.return_to_launch()
    await asyncio.sleep(20)

    # 降落
    await drone.action.land()
```

**优点**：简单可靠，飞控内部闭环，不需要我们维护 setpoint 发送循环。
**缺点**：控制精度受飞控参数影响（默认 GPS 导航精度 ~1m），不够灵活。

#### 2. Offboard 插件 — 直接位置/速度控制

最灵活的控制方式。程序以 ≥2Hz 的频率持续发送 setpoint，飞控执行。支持三种 setpoint：
- **位置**：`set_position_ned(PositionNedYaw(north_m, east_m, down_m, yaw_deg))` — NED 坐标系
- **速度**：`set_velocity_ned(VelocityNedYaw(north_m_s, east_m_s, down_m_s, yaw_deg))`
- **加速度**：`set_acceleration_ned(AccelerationNed(…))`

```python
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
import asyncio

async def offboard_flight():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # 等待连接就绪
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    print("-- Arming")
    await drone.action.arm()

    # 必须先 set 一个 setpoint 才能 start offboard
    print("-- Starting offboard mode")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print("Offboard failed, disarming")
        await drone.action.disarm()
        return

    # 持续发送 setpoint（关键：必须保持 ≥2Hz 频率，否则飞控会超时退出 offboard）
    print("-- Fly NED: 30m forward, maintain altitude")
    for _ in range(100):  # 约 10 秒 @ 10Hz
        await drone.offboard.set_position_ned(PositionNedYaw(30.0, 0.0, -7.0, 0.0))
        await asyncio.sleep(0.1)

    print("-- Stop offboard and land")
    await drone.offboard.stop()
    await drone.action.land()

asyncio.run(offboard_flight())
```

**坐标系说明（NED）**：

NED 是一个**世界固定坐标系**（local tangent plane），不是随机体旋转的坐标系。

- **N**（North）：**真北方向**，不是机头朝向。由 GPS/磁罗盘确定，与飞行器朝向无关
- **E**（East）：**真东方向**，与 N 轴正交
- **D**（Down）：沿重力方向向下（正值向下）
- **原点 (0, 0, 0)**：Home 点——飞控**上锁（arm）时刻的 GPS 位置**自动记录为 Home
- **yaw_deg**：独立于位置的偏航角，0° = 指向正北，顺时针增加（俯视视角）

> `PositionNedYaw(north_m, east_m, down_m, yaw_deg)` 中四个字段相互独立。飞行器飞往 (10, 0, -7) 的同时可以 yaw=90°（面朝东），不影响飞行路径。

**Home 点如何设置**：

Home 点由 PX4 飞控在 **arm（上锁解锁）时刻** 自动记录当前的 GPS 经纬度 + 海拔，作为本地 NED 坐标系的原点。这意味着：
- 放飞行器到起降点 → arm → 那一刻的位置就是 (0, 0, 0)
- 之后的 NED 坐标全部相对于这个点
- 不需要手动设置，飞控自动完成

**坐标系转换问题**：

由于 N 轴始终指向真北，而比赛场地的"前方"（起降点 → 投放区 → 侦察区）通常不严格沿正北方向，需要一个转换层。代码中所有任务坐标均在"场地 NED"坐标系中定义，通过旋转矩阵映射到真北 NED 后发送给 PX4。

```
场地 NED 坐标系              真北 NED 坐标系（PX4 使用）
    ↑ N_f (场地前方)              ↑ N (真北)
    |                           /
    |                          /
    |                         / θ = FIELD_YAW_DEG
    |                        /
    +--→ E_f (场地右方)        +----→ E (真东)

场地前方 ≠ 真北，相差一个角度 θ
```

```python
import math

def field_to_ned(north_m: float, east_m: float, up_m: float,
                 field_yaw_deg: float) -> tuple:
    """
    将场地对齐的 NED 坐标转换为真北 NED 坐标。

    旋转矩阵：
        [true_N]   [cos(θ)  -sin(θ)] [north_m]
        [true_E] = [sin(θ)   cos(θ)] [east_m]

    其中 θ = field_yaw_deg（场地前方方向的真北方位角）。

    Args:
        north_m: 场地 NED 北向分量（沿场地前方，m）
        east_m:  场地 NED 东向分量（沿场地右方，m）
        up_m:    飞行高度（m，向上为正）
        field_yaw_deg: 场地前方方向对应的真北方位角（°）

    Returns:
        (true_north_m, true_east_m, down_m)
    """
    theta = math.radians(field_yaw_deg)
    true_north_m = north_m * math.cos(theta) - east_m * math.sin(theta)
    true_east_m  = north_m * math.sin(theta) + east_m * math.cos(theta)
    down_m  = -up_m   # NED 中 down 正值向下，高度取负
    return true_north_m, true_east_m, down_m
```

**实际使用流程**：

1. 赛前使用指南针/手机罗盘测量场地前方方向的真北方位角
2. 将测量值填入 `config.py` 的 `FIELD_YAW_DEG` 参数
3. 程序启动时从 config 读取，不再依赖飞行器航向自动检测
4. Arm → PX4 自动记录 Home 点 = (0, 0, 0)
5. 使用 `field_to_ned()` 将场地 NED 坐标转为真北 NED 坐标发给 offboard

```python
# 示例：场地前方朝向北偏东 15°
FIELD_YAW = 15.0  # 赛前手动测量并填入 config.py

# 飞到投放区起点（场地北 30m, 高度 7m）
true_n, true_e, true_d = field_to_ned(30.0, 0.0, 7.0, FIELD_YAW)
await drone.offboard.set_position_ned(PositionNedYaw(true_n, true_e, true_d, FIELD_YAW))
```

> `FIELD_YAW_DEG` 是赛前在 `config.py` 中手动配置的参数，不再从飞行器航向自动检测。这避免了磁罗盘误差对场地朝向判断的影响。

#### 补充：TransitState 与 MAVSDK 的对应关系
`TransitState` 的参数 `(north, east, up, speed)` 直接映射到 MAVSDK offboard 控制：

| TransitState 参数 | 含义        | MAVSDK 映射                                           |
| --------------- | --------- | --------------------------------------------------- |
| `north`         | 场地 NED 北向分量（沿场地前方，m） | `PositionNedYaw.north_m`（经旋转后）           |
| `east`          | 场地 NED 东向分量（沿场地右方，m） | `PositionNedYaw.east_m`（经旋转后）             |
| `up`            | 目标高度（m）   | `PositionNedYaw.down_m`（= -up_m，经旋转后）      |
| `speed`         | 巡航速度（m/s） | 可选：`set_velocity_ned()` 或设置 PX4 参数 `MPC_XY_VEL_MAX` |

**实现方式有两种**：

1. **Offboard set_position_ned（推荐）**：先通过 `field_to_ned()` 旋转坐标，再发送 setpoint，飞控内部自动生成速度曲线
2. **action.goto_location**：如果已知目标 GPS 坐标

**示例代码**：

```python
async def transit_to(drone, north, east, up, speed, timeout=20):
    """TransitState 实现：以指定速度飞到场地 NED 目标位置"""
    # 先通过旋转矩阵转换为真北 NED
    true_n, true_e, true_d = field_to_ned(north, east, up, FIELD_YAW)
    target = PositionNedYaw(true_n, true_e, true_d, 0.0)
    target_norm = (north**2 + east**2 + up**2) ** 0.5
    duration = max(target_norm / speed, 2)  # 预估飞行时间

    print(f"Transit: target=({north},{east},{up}), speed={speed}m/s, est={duration:.1f}s")
    start = asyncio.get_event_loop().time()

    while True:
        elapsed = asyncio.get_event_loop().time() - start
        if elapsed > duration + 5:  # 超时退出
            break

        await drone.offboard.set_position_ned(target)
        await asyncio.sleep(0.1)  # 10Hz 发送频率

        # 检查是否到达（可选）
        async for pos in drone.telemetry.position():
            dist_n = abs(pos.relative_altitude_m - up)  # 简化
            # ... 到达判定逻辑
            break

    print("Transit complete")
```

#### 3. Telemetry 插件 — 读取飞控状态

实时流式获取飞控遥测数据。

```python
async def monitor_telemetry(drone):
    # 位置
    async for pos in drone.telemetry.position():
        print(f"lat={pos.latitude_deg:.6f} lon={pos.longitude_deg:.6f} "
              f"alt={pos.relative_altitude_m:.2f}m")

    # 速度 (NED)
    async for vel in drone.telemetry.velocity_ned():
        print(f"Vn={vel.north_m_s:.1f} Ve={vel.east_m_s:.1f} Vd={vel.down_m_s:.1f}")

    # 电池
    async for bat in drone.telemetry.battery():
        print(f"Battery: {bat.remaining_percent:.1f}% ({bat.voltage_v:.2f}V)")

    # 飞行模式
    async for mode in drone.telemetry.flight_mode():
        print(f"Flight mode: {mode}")

    # 是否在空中
    async for in_air in drone.telemetry.in_air():
        print(f"In air: {in_air}")

    # 着陆状态 (UNKNOWN, ON_GROUND, IN_AIR, TAKING_OFF, LANDING)
    async for ls in drone.telemetry.landed_state():
        print(f"Landed state: {ls}")
```

#### 4. Param 插件 — 读写飞控参数

```python
# 读取参数
nav_acc = await drone.param.get_param_float("NAV_ACC_RAD")

# 设置参数
await drone.param.set_param_int("MPC_Z_VEL_MAX_DN", 1)  # 最大下降速度
```

关于PX4参数列表，详见[[PX4参数列表与介绍]]
#### 5. Action 插件 — 舵机/执行器控制

```python
# set_actuator(index, value) — index 从 1 开始，value 范围 [-1, 1]
# 例如：通过飞控的 AUX 输出控制舵机
await drone.action.set_actuator(1, 1.0)   # 舵机释放
await drone.action.set_actuator(1, -1.0)  # 舵机复位
```

### 控制方式对比总结

| 控制方式 | 接口 | 精度 | 复杂度 | 适用场景 |
|---------|------|------|--------|----------|
| Action `goto_location` | GPS 坐标 | ~1m | 低 | 航线飞行、粗定位 |
| Action `takeoff` / `land` | 自动流程 | 中等 | 低 | 起飞/降落 |
| Offboard `set_position_ned` | NED 坐标系 | 取决于定位精度 | 中 | 精准悬停、视觉伺服微调 |
| Offboard `set_velocity_ned` | NED 速度 | 灵活 | 中 | 视觉伺服动态调整 |
| `set_actuator` | 飞控 AUX 输出 | — | 低 | 舵机/投放机构控制 |

### 本项目中的使用策略

- **起飞**：`action.set_takeoff_altitude()` + `action.takeoff()`（简单可靠）
- **航线飞行**（投放/侦察区之间）：offboard `set_position_ned`（精准控制位置）
- **投放对准**：offboard `set_velocity_ned` 或 `set_position_ned` + 视觉伺服 PID
- **降落**：先在 offboard 中降落到 ~0.5m → `action.land()`
- **舵机释放**：`action.set_actuator(index, value)`

---

## asyncio 并发机制

MAVSDK Python 完全基于 asyncio，理解其并发模型对于写出正确的飞控程序至关重要。

### 为什么需要 asyncio

飞行任务同时运行多项工作，它们不能互相阻塞：

```
时间线需求（单线程内交替执行）:
├─ 10Hz 持续发送 offboard setpoint    ← 不能中断，否则飞控超时退出 offboard
├─ 等待飞控返回遥测数据               ← I/O 等待，可能几十ms
├─ 摄像头抓帧 + YOLO 推理             ← 耗时 50-200ms
└─ 状态机逻辑（判断阶段切换）          ← 随时响应
```

如果用同步代码，一个 `time.sleep()` 或 `cv2.imread()` 就卡住一切。asyncio 的核心思想是**单线程 + 事件循环**：遇到 I/O 等待时切换到其他任务，而不是阻塞整条线程。

### 核心概念

**`async def`** — 声明协程函数。调用它不执行，只返回一个 coroutine 对象。

**`await`** — "我在这等着，但你可以先去干别的"。注意：只有 await 后面是真正的异步操作时才会让出控制权；如果 await 的是同步代码，它仍然会阻塞。

**`asyncio.create_task()`** — 把协程包装成 Task 丢进事件循环并发运行。

**`asyncio.run()`** — 启动事件循环，是程序的入口。

```
asyncio 事件循环 (Event Loop)
    │
    ├── Task A: setpoint_sender()      ── await asyncio.sleep(0.1) 时让出
    ├── Task B: telemetry_watcher()    ── async for 等待飞控数据时让出
    └── Task C: vision_servo()         ── await 图像处理结果时让出
```

### 关键陷阱

**1. CPU 密集操作会阻塞整个事件循环**

OpenCV 图像处理、YOLO 推理都是同步 CPU 密集操作，直接 await 会把所有其他任务卡死：

```python
# ❌ 错误：YOLO 推理阻塞 setpoint 发送，飞控可能超时退出 offboard
async def bad_vision():
    frame = camera.read()          # 同步 I/O
    results = yolo_model(frame)    # CPU 密集，阻塞所有 asyncio Task
```

```python
# ✅ 正确：把耗时操作丢到线程池
import concurrent.futures

executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)

async def good_vision():
    loop = asyncio.get_running_loop()
    frame = await loop.run_in_executor(executor, camera.read)
    results = await loop.run_in_executor(executor, yolo_model, frame)
```

**2. `await` 不会让代码并行，只会让出等待**

```python
# 这三行顺序执行，和同步代码没区别
await task_a()   # 执行完
await task_b()   # 才执行
await task_c()   # 最后执行

# 要并发，必须用 create_task
t1 = asyncio.create_task(task_a())
t2 = asyncio.create_task(task_b())
await t1   # t1 和 t2 交替跑
await t2
```

**3. `asyncio.sleep()` 是最小让出单位**

在 offboard 发送循环中，`await asyncio.sleep(0.1)` 不仅是定时，更关键的是**让出控制权**给其他 Task。如果没有这个 sleep 或其他 await，其他 Task 永远得不到执行。

### 本项目典型并发模式

```python
import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

async def main():
    drone = System()
    await drone.connect("udp://:14540")
    # … 等待连接、GPS、arm、PX4内建起飞、进入 offboard …

    # 共享变量（asyncio 单线程，不需要锁）
    latest_cmd = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)

    async def setpoint_sender():
        """Task 1: 持续发送速度指令（10Hz），雷打不动"""
        while True:
            await drone.offboard.set_velocity_ned(latest_cmd)
            await asyncio.sleep(0.1)  # 10Hz，唯一让出点

    async def vision_loop():
        """Task 2: 视觉伺服（按自己节奏跑，20Hz）"""
        nonlocal latest_cmd
        while True:
            loop = asyncio.get_running_loop()
            frame = await loop.run_in_executor(executor, camera.read)
            offset_x, offset_y = compute_offset(frame)
            latest_cmd = VelocityNedYaw(
                Kp * offset_x, Kp * offset_y, 0.0, 0.0
            )
            await asyncio.sleep(0.05)  # 20Hz

    t1 = asyncio.create_task(setpoint_sender())
    t2 = asyncio.create_task(vision_loop())
    await asyncio.gather(t1, t2)

asyncio.run(main())
```

> 关键：setpoint 发送 10Hz、视觉处理 20Hz，两者在同一线程交替运行。视觉推理通过 `run_in_executor` 丢到线程池，主协程不会被卡住。因为是单线程，`latest_cmd` 的读写天然线程安全，无需加锁。