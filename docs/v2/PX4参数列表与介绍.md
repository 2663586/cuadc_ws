# PX4 参数列表与介绍

本项目基于 PX4 v1.16.1，以下列出程序中涉及的所有关键飞控参数。程序侧全局参数见 [[机载电脑程序#零、全局关键参数|机载电脑程序 §零]]。

---

## 一、Offboard 控制相关

### `COM_OF_LOSS_T`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | -1 (disabled) |
| 推荐值 | 默认 |
| 说明 | Offboard 模式断连超时（秒）。若飞控在此时间内未收到 MAVLink setpoint 消息，将自动退出 Offboard 并触发保护（Hold 或 Land）。程序通过 [[机载电脑程序#3.2 Offboard 心跳维持机制|心跳机制]] 以 20Hz 发送 setpoint 规避此问题。 |

### `COM_RCL_EXCEPT`

| 属性 | 值 |
|------|-----|
| 类型 | int (bitmask) |
| 默认值 | 0 |
| 推荐值 | 4 (bit 2 = Offboard) |
| 说明 | RC 信号丢失时允许保持的模式。设置 bit 2 允许 RC 丢失后继续保持 Offboard 模式。比赛期间遥控器只用于紧急接管，需设置此参数防止自动切模式。 |

---

## 二、速度/位置控制

### `MPC_XY_VEL_MAX`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | ~12 m/s |
| 推荐值 | `8.0` |
| 说明 | 最大水平飞行速度（m/s）。需 ≥ 程序侧 `TRANSIT_SPEED_MPS`（默认 5.0）。设置过高可能导致投放和侦察阶段难以稳定悬停。 |
| 引用 | [[MAVSDK简介#TransitState 与 MAVSDK 的对应关系\|TransitState speed 参数]] |

### `MPC_Z_VEL_MAX_DN`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | 1.5 m/s |
| 推荐值 | `1.5` |
| 说明 | 最大下降速度（m/s）。投放阶段从 7m → 2.5m 以及着陆阶段均依赖此参数。设置过小会导致下降耗时过长，过大则着陆冲击大。 |
| 引用 | [[MAVSDK简介#4. Param 插件\|Param 插件示例]] |

### `MPC_Z_VEL_MAX_UP`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | 3.0 m/s |
| 推荐值 | `3.0` |
| 说明 | 最大爬升速度（m/s）。投放完第一瓶后需要从 ~2.5m 爬升至 ~3.5m 再平移至第二个目标。 |

### `MPC_XY_P`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | 0.95 |
| 推荐值 | 默认 |
| 说明 | 水平位置控制 P 增益。增加此值可提高位置跟踪响应速度，但过高会导致振荡。视觉伺服阶段此参数影响对准稳定性。 |

---

## 三、起飞/着陆

### `MIS_TAKEOFF_ALT`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | 2.5 m |
| 推荐值 | `7.0` |
| 说明 | 默认起飞高度（m）。PX4 `action.takeoff()` 的目标高度，应与程序侧 `CRUISE_ALTITUDE_M` 一致。程序在 `interface.takeoff()` 中调用。 |

### `COM_DISARM_LAND`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | 2.0 s |
| 推荐值 | 默认 |
| 说明 | 着陆后自动上锁延时（秒）。着陆成功后螺旋桨停止旋转才算有效着陆（着陆分要求）。此参数需 >0 确保自动 disarm。 |

### `MPC_LAND_SPEED`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | 0.7 m/s |
| 推荐值 | `0.5` |
| 说明 | AUTO.LAND 模式下的下降速度（m/s）。降低此值可减小着陆冲击，但增加着陆耗时。精确着陆阶段在视觉兜底方案中会用到。 |

---

## 四、导航

### `NAV_ACC_RAD`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | 10.0 m |
| 推荐值 | 默认 |
| 说明 | 导航到达判定半径（m）。`action.goto_location()` 中飞控认为"已到达"的距离阈值。本项目主要使用 offboard `set_position_ned`，此参数影响有限。 |
| 引用 | [[MAVSDK简介#4. Param 插件\|Param 插件示例]] |

---

## 五、定位/姿态估计

### `EKF2_AID_MASK`

| 属性 | 值 |
|------|-----|
| 类型 | int (bitmask) |
| 默认值 | 取决于硬件 |
| 推荐值 | 视传感器配置 |
| 说明 | EKF2 辅助模式掩码。bit 1 = GPS, bit 2 = optical flow, bit 3 = vision position, bit 4 = vision yaw。若使用视觉定位（如 VIO），需启用相关 bit。 |
| 备注 | 程序侧 `global_guard_check()` 通过 `estimator_flags_ok` 监控 EKF 状态，若 RTK 精度突增将触发保护。 |

### `EKF2_HGT_MODE`

| 属性 | 值 |
|------|-----|
| 类型 | int |
| 默认值 | 0 (barometric) |
| 推荐值 | 0 或 2 (range sensor, 若有激光/超声波定高) |
| 说明 | 高度估计来源。投放和侦察阶段的低空飞行（2-3m）对高度精度要求高，若有测距传感器建议设置为 2。 |

---

## 六、舵机/执行器

### `SYS_USE_IO`

| 属性 | 值 |
|------|-----|
| 类型 | int |
| 默认值 | 1 (enable) |
| 推荐值 | 1 |
| 说明 | 启用 IO 板。舵机通过飞控 AUX 输出口控制，需确保 IO 板启用。`set_actuator(1, value)` 指令依赖此配置。 |

### AUX 通道参数（`CA_AUX_0`, `CA_AUX_1` 等）

| 属性 | 值 |
|------|-----|
| 类型 | int (function ID) |
| 推荐值 | `Set_actuator = 206` |
| 说明 | 将 AUX 通道的功能设置为 "Custom Actuator"，使 `set_actuator(index, value)` 能映射到对应的物理输出口。需在 PX4 参数中逐一配置用到的 AUX 通道。 |

---

## 七、电池/安全

### `BAT_LOW_THR`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | 0.15 (15%) |
| 推荐值 | `0.2` (20%) |
| 说明 | 低电量警告阈值（归一化电压）。应与程序侧 `BATTERY_LOW_THRESHOLD_PCT` 一致。飞控和程序双重监控。 |

### `BAT_CRIT_THR`

| 属性 | 值 |
|------|-----|
| 类型 | float |
| 默认值 | 0.07 (7%) |
| 推荐值 | `0.1` (10%) |
| 说明 | 严重低电量阈值。低于此值飞控将强制降落。比赛中应确保此值不会在正常任务中被触发。 |

---

## 八、参数读写方式

程序通过 MAVSDK 的 Param 插件读写参数：

```python
# 读取
value = await drone.param.get_param_float("MPC_XY_VEL_MAX")

# 设置（int 型参数）
await drone.param.set_param_int("MIS_TAKEOFF_ALT", 7)

# 设置（float 型参数）
await drone.param.set_param_float("MPC_XY_VEL_MAX", 8.0)
```

也可通过 QGroundControl → Parameters 界面、或 PX4 启动脚本（`/etc/extras.txt`）预设。

> 比赛规则要求全程自主，不允许人工操纵。所有飞控参数应在赛前通过配置脚本批量写入，赛中不应通过 QGC 手动调整。

---

## 关联笔记

- [[机载电脑程序]] — 程序侧全局参数与状态机设计
- [[MAVSDK简介]] — MAVSDK Param 插件用法
