## MAVROS相关

在开发基于 ROS (Robot Operating System) 的无人机系统时，理解 **MAVROS 话题**、**ROS 消息类型** 与 **MAVLink 消息** 之间的映射关系是核心功课。简单来说，MAVROS 扮演的是一个“同声传译”的角色。

以下是它们之间关系的深度解析：
### 1. 核心架构：三位一体的转换模型

这三者的关系可以看作一个数据流管道：

1. **无人机端 (PX4/ArduPilot)**：产生 **MAVLink** 二进制数据包。

2. **MAVROS 节点**：拦截这些包，将其解析并封装进 **ROS Message** 中。

3. **开发者端 (ROS Node)**：订阅对应的 **ROS Topic** 来获取状态。

### 关系映射表

| 层面           | 表现形式                 | 特点                          |
| ------------ | -------------------- | --------------------------- |
| MAVLink      | 二进制流 (Serial/UDP)    | 轻量、高效，适合带宽有限的电台或串口。         |
| ROS Message  | 结构化对象 (.msg)         | 强类型，方便在 ROS 生态内进行数学运算和逻辑处理。 |
| MAVROS Topic | 命名空间路径 (/mavros/...) | 数据的入口和出口，实现发布/订阅机制。         |

### 2. 数据流向示例：以“无人机姿态”为例

为了让你看得更清晰，我们追踪一下无人机当前的姿态（Orientation）是如何传递的：

#### A. MAVLink 层

无人机飞控通过串口发送一个 ID 为 `#30` 的消息：`ATTITUDE`。它包含 `roll`, `pitch`, `yaw` 等原始弧度数据。

#### B. MAVROS 转换层

MAVROS 内部的 `sys_status` 或 `imu` 插件会监听这个 ID。它不仅做格式转换，还会进行**坐标系转换**（将无人机常用的 **NED** 坐标系转换为 ROS 标准的 **ENU** 坐标系）。

#### C. ROS 层

最终，你在终端输入 `rostopic echo /mavros/imu/data` 看到的数据，其消息类型是 `sensor_msgs/Imu`。

### 3. 话题与消息类型的深度绑定

在 MAVROS 中，并不是每一个 MAVLink 消息都对应一个唯一的话题，有时一个 ROS 话题会整合多个 MAVLink 消息。

#### 常见的映射案例：


1. **状态信息**

  - **话题**: `/mavros/state`

  - **ROS 消息**: `mavros_msgs/State`

  - **MAVLink 来源**: `HEARTBEAT` (#0)

  - *用途*: 查看是否连接、是否解锁 (Armed)、当前飞行模式 (OFFBOARD/AUTO)。

2. **GPS 经纬度**

  - **话题**: `/mavros/global_position/global`

  - **ROS 消息**: `sensor_msgs/NavSatFix`

  - **MAVLink 来源**: `GLOBAL_POSITION_INT` (#33)

  - *用途*: 地图定位、全球路径规划。

3. **控制指令 (反向流)**

  - **话题**: `/mavros/setpoint_raw/local`

  - **ROS 消息**: `mavros_msgs/PositionTarget`

  - **MAVLink 转换**: `SET_POSITION_TARGET_LOCAL_NED` (#84)

  - *用途*: ROS 发送目标点给飞控执行。

### 4. 关键点：坐标系的“隐形”转换

这是初学者最容易踩坑的地方。

- **MAVLink** 遵循航空标准：**NED** (North-East-Down) 坐标系。

- **ROS** 遵循机器人标准：**ENU** (East-North-Up) 坐标系。

**MAVROS 的功劳**：当你向 `/mavros/setpoint_position/local` 发布一个 $Z=2.0$ 的消息时，MAVROS 会自动将其转换为 MAVLink 中的 $D=-2.0$。你不需要手动去改正负号，只要按照 ROS 的习惯操作即可。
### 5. 如何查看具体的映射关系？

如果你想知道某个特定话题对应的 MAVLink 源码，可以参考以下路径：

1. **源码查找**：在 MAVROS 的源码目录 `mavros/src/plugins/` 下，每一个插件（如 `local_position.cpp`）都详细记录了它订阅了哪个 MAVLink ID，以及发布了哪个 ROS Topic。

2. **运行时查看**：

  - 查看消息结构：`rosmsg show mavros_msgs/State`

  - 查看话题类型：`rostopic type /mavros/local_position/pose`

提示：并非所有 MAVLink 消息都被 MAVROS 默认转发。如果你自定义了 MAVLink 消息，你需要自己编写一个 MAVROS 插件（Plugin）来完成这个映射。

## 关心话题

