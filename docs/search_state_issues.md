# Search 状态伪代码分析与待解决问题

> 基于 `src/search(伪代码).py` 的分析，对比当前 `src/states/search.py`、`src/main_fsm.py`、`config.py`。

---

## 架构概览

伪代码将 SearchState 从"悬停单次检测"重构为"矩形航线 + 边飞边检测 + 栈式抢占"，核心流程：

```
enter → 初始化矩形四顶点航点
  ↓
execute 循环:
  1. 超时检查
  2. 飞向当前航点 (_fly_to_target)
  3. 视觉检测 (YOLO → Canny → HoughCircles → 直径)
  4. 匹配到瓶子 → interrupt=AlignState (栈式抢占)
  5. 到达航点 → 推进到下一个航点 (循环绕圈)
```

栈式抢占流程：

```
SearchState 检测到目标
  → return ExecutionResult(interrupt=AlignState(bottle_index=N))
    → main_fsm.py: SearchState.suspend() → _stack.append(AlignState)
      → AlignState 执行 (对准 + 投放)
      → AlignState done → 弹出 → SearchState.resume() 继续搜索第二个瓶子
```

---

## 待解决问题清单

### 🔴 P0 — 阻塞性（不解决无法运行）

| # | 问题 | 所在位置 | 说明 |
|---|------|---------|------|
| 1 | **矩形航点坐标未定义** | `search(伪代码).py:29-32` | `P1_x`~`P4_y` 是字面占位符变量，代码会 `NameError`。需决定是硬编码四个顶点还是用中心+长宽计算 |
| 2 | **`epsilon` 未定义** | `search(伪代码).py:59,66` | 直径匹配容差变量不存在，代码会 `NameError` |
| 3 | **心跳/Offboard 冲突** | `search(伪代码).py:90` | `_fly_to_target()` 绕过 `update_setpoint()` 直接调 `set_position_ned()`，会与后台心跳发送的 setpoint 交替覆盖，导致飞机抖动或偏航。伪代码自注释"后续会修改心跳代码以配合此模式" |

### 🟡 P1 — 设计问题（能跑但行为不对）

| # | 问题 | 所在位置 | 说明 |
|---|------|---------|------|
| 4 | **检测结果单位混淆（像素 vs 米）** | `search(伪代码).py:108-109` | `TargetStub.ned_offset` 存的是 `(cx_px, cy_px)` 像素坐标，但 `align.py:49-52` 期望 `ned_offset` 是米制 NED 偏移，拿去加 `DROP_ZONE_DISTANCE_M`。单位不匹配 |
| 5 | **边飞边检测 → 运动模糊** | `search(伪代码).py:43-51` | 先发飞行指令不等到达就做视觉推理，YOLO+HoughCircles 在运动模糊下检测率显著下降 |
| 6 | **相同直径瓶子无法区分** | `search(伪代码).py:59,66` | `goal[0]`/`goal[1]` 的检查逻辑假设瓶子1=15cm、瓶子2=20cm。若两个瓶子直径相同，第二个永远不会触发抢占 |
| 7 | **Resume 后无拉升逻辑** | `search(伪代码).py:64` | AlignState 完成时无人机在 ~2m 低空，resume 后直接飞向巡航高度的矩形航点，会从低空斜向拉升，可能擦碰障碍物 |

### 🟢 P2 — 健壮性问题（能跑但不可靠）

| # | 问题 | 所在位置 | 说明 |
|---|------|---------|------|
| 8 | **单帧检测不可靠** | `search(伪代码).py:48-51` | 每周期只处理一帧，漏检则需绕一整圈才能再看。应连续多帧确认 |
| 9 | **视觉流水线延迟 > FSM 周期** | `search(伪代码).py:51` | YOLO→Canny→HoughCircles 一帧可能 50-200ms，FSM 周期 50ms (20Hz)，推理会落后于实时 |
| 10 | **无避障** | 整体 | 矩形航线飞行中不做任何避障检查 |
| 11 | **`_capture_frame` 同步调用** | `search(伪代码).py:48` | 伪代码写 `_capture_frame()`（同步），在 async 上下文中会阻塞事件循环。实际代码用 `_capture_frame_async()` 在线程池运行 |

### 🔵 P3 — 配置缺失（需要加到 config.py）

| # | 超参数 | 建议默认值 | 说明 |
|---|--------|-----------|------|
| 12 | `SEARCH_SPEED_MPS` | 3.0 | 矩形航线飞行速度 |
| 13 | `ARRIVAL_THRESHOLD_M` | 1.0 | 航点到达判定距离 |
| 14 | `SEARCH_RECT_CENTER_N` / `_E` | 待定 | 矩形中心坐标（北/东），替代硬编码 P1~P4 |
| 15 | `SEARCH_RECT_LENGTH` / `WIDTH` | 待定 | 矩形长宽，配合中心坐标自动算四顶点 |
| 16 | `BOTTLE_DIAMETER_TOLERANCE_CM` | 2.0 | 瓶子直径匹配容差（替代 `epsilon`） |
| 17 | `CIRCLE_CONF_THRESHOLD` | 0.3 | HoughCircles 的最低 YOLO 置信度 |
| 18 | `YOLO_MODEL_PATH` | `"models/yolov11n_800_best_FP16.engine"` | YOLO 模型路径 |
| 19 | `SEARCH_STATE_TIMEOUT_S` | 120 | 搜索状态整体超时（当前 search.py 是 30s，伪代码 120s） |

---

## 解决顺序建议

```
P0-1 (航点坐标) → P0-2 (epsilon) → P0-3 (心跳冲突)
  → P1-4 (单位混淆) → P1-5 (运动模糊) → P1-6 (相同直径) → P1-7 (拉升)
    → P3 (config 补齐)
      → P2-8 (多帧确认) → P2-9 (流水线延迟) → P2-11 (capture_frame)
        → P2-10 (避障，长期)
```

---

## 相关文件

| 文件 | 角色 |
|------|------|
| `src/search(伪代码).py` | 本分析的源文件（伪代码） |
| `src/states/search.py` | 当前实际 SearchState（悬停+单次检测） |
| `src/states/align.py` | AlignState，被抢占压入的目标状态 |
| `src/states/base_state.py` | BaseState / ExecutionResult 定义 |
| `src/main_fsm.py` | 栈式 FSM 引擎，处理 interrupt/done/suspend/resume |
| `config.py` | 全局参数，需要补齐 P3 各项 |
| `src/vision/pipeline.py` | VisionPipeline，YOLO+Canny+HoughCircles |
