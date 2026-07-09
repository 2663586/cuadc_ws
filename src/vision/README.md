# Vision 模块

Jetson Orin NX 机载视觉流水线：**YOLO 目标检测 → 圆形边缘提取 → 圆心定位 → 真实直径估算**。

## 文件结构

```
src/vision/
├── yolo_detector.py          # YOLO 推理封装 (TensorRT)
├── circle_detector.py        # 圆检测 + 针孔相机模型
├── pipeline.py               # 流水线串联 (YOLO → Circle)
├── circle_detect_pipeline.py # 原始离线批处理脚本 (参考)
├── models/
│   └── yolov11n_800_best_FP16.engine   # TensorRT FP16 模型权重
└── __init__.py
```

## 核心组件

### 1. YOLODetector (`yolo_detector.py`)

加载 TensorRT engine，对单帧图像进行目标检测。

- **模型**: `yolo11n-800`，输入尺寸 800×800，FP16 推理
- **类别**: `0` = 蓝色背景板, `1` = 桶 (投放目标)
- **性能**: ~5.4ms/帧 纯推理 (TensorRT FP16 @ Orin NX)

```python
from vision import YOLODetector

detector = YOLODetector("models/yolov11n_800_best_FP16.engine", conf=0.5)
dets = detector.detect(frame)
# → [{"x1": 416, "y1": 289, "x2": 468, "y2": 347, "conf": 0.892, "cls": 1, "name": "bucket"}, ...]

# 只取桶 (class=1) 的检测
buckets = detector.get_class1_detections(dets, conf_threshold=0.3)
```

### 2. CircleDetector (`circle_detector.py`)

在 YOLO 检测框 ROI 内进行 Canny 边缘检测 + HoughCircles 圆检测，并用针孔相机模型计算真实直径。

```python
from vision import CircleDetector, CircleResult

cd = CircleDetector(
    camera_matrix=...,    # 3×3 内参矩阵 (有默认值)
    bucket_height_m=0.30, # 桶的物理高度
)

# 检测圆
result: CircleResult = cd.detect(frame, bbox=(x1, y1, x2, y2))
print(result.cx_px, result.cy_px, result.radius_px)  # 圆心 + 半径 (像素)

# 计算真实直径
diameter_m = cd.compute_diameter(result.radius_px, alt_rel_m=7.0)

# 可视化叠加
edges = cd.get_edges(roi)
annotated = cd.overlay(frame, roi_x1, roi_y1, edges, result)
```

**检测流程**:
```
YOLO bbox → ROI 提取(+20% padding) → 灰度化 → 高斯模糊(5×5)
    → Canny 边缘(40/120) → HoughCircles(dp=1.2, minDist=20)
    → 选最大半径圆 → 针孔模型计算直径
```

**针孔模型公式**:

```
Z_C = alt_rel_m - bucket_height_m     # 相机到桶顶距离
D_real = (2 × r_px × Z_C) / fx        # 真实直径 (米)
```

### 3. VisionPipeline (`pipeline.py`)

串联 YOLO → Circle，提供统一接口，支持实时和离线两种模式。

```python
from vision import VisionPipeline

pl = VisionPipeline(
    model_path="models/yolov11n_800_best_FP16.engine",
    yolo_conf=0.5,
    circle_conf_threshold=0.3,
    apply_undistort=False,      # 是否启用畸变校正
    max_infer_fps=15,           # 推理帧率限制 (0=不限)
)

# ── 实时模式: 逐帧处理 ──
results = pl.process_frame(frame, alt_rel_m=7.0)
for r in results:
    print(r["det"]["conf"])        # YOLO 置信度
    print(r["circle"].cx_px)       # 圆心 x (全帧坐标)
    print(r["diameter_m"])         # 真实直径 (米)

# ── 离线模式: 批量处理录制视频 ──
pl.process_video(
    video_path="recording.mp4",
    log_csv_path="log.csv",        # 含 alt_rel_m 和 monotonic_ns
    output_dir="output/",
    output_video=True,             # 生成带叠加的输出视频
)
```

## 命令行用法

```bash
# 离线处理录制视频
python -m vision.pipeline \
    --video dataset_recorder/recordings/20260701_153306/20260701_153306.mp4 \
    --log   dataset_recorder/recordings/20260701_153306/log.csv \
    --output output/circle_detect \
    --undistort \
    --fps-limit 15

# 参数说明:
#   --video     输入视频路径 (必需)
#   --log       log.csv 遥测日志路径 (必需)
#   --output    输出目录 (默认: output/circle_detect)
#   --model     YOLO 模型路径 (默认: models/yolov11n_800_best_FP16.engine)
#   --no-video  不生成叠加视频
#   --undistort 启用畸变校正
#   --frames    最大处理帧数 (0=全部)
#   --fps-limit 推理帧率限制 (0=不限)
```

## 输出格式

离线模式输出两个文件：

### `{视频名}_circle_detect.csv`

| 列名 | 说明 |
|------|------|
| `frame_idx` | 视频帧序号 (0-based) |
| `monotonic_ns` | 单调时钟纳秒 (与 log.csv 对齐) |
| `alt_rel_m` | 相对高度 (米) |
| `det_idx` | 帧内检测序号 |
| `conf` | YOLO 置信度 |
| `bbox_x1,y1,x2,y2` | YOLO 边界框 (像素) |
| `edge_success` | 圆检测是否成功 |
| `circle_cx_px, cy_px` | 圆心全帧坐标 (像素) |
| `circle_radius_px` | 圆半径 (像素) |
| `circle_diameter_px` | 圆直径 (像素) |
| `circle_diameter_m` | 估算真实直径 (米) |

### `{视频名}_circle_overlay.mp4`

叠加了 Canny 边缘 (半透明绿色) + 检测圆 (红色) + 圆心十字线 + 直径标注的输出视频。

## 相机内参

默认使用 Orin NX 下视摄像头棋盘格标定结果 (1280×720):

```
fx = 1059.15   fy = 1058.86
cx = 638.37    cy = 355.42
k1 = 0.0343   k2 = -0.0376   p1 = 0.0034   p2 = 0.0025   k3 = -0.0175
平均重投影误差: 0.1784 px
```

可通过 `CircleDetector(camera_matrix=..., dist_coeffs=...)` 覆盖。

## 在飞行状态机中使用

```python
# 在 Search/Align 等状态中集成
from vision import VisionPipeline

class SearchState(BaseState):
    def on_enter(self):
        self.pipeline = VisionPipeline(max_infer_fps=15)

    async def loop(self, dt):
        frame = self.camera.read()          # 获取当前帧
        alt = self.interface.alt_rel_m      # 当前高度

        results = self.pipeline.process_frame(frame, alt_rel_m=alt)
        for r in results:
            if r["edge_success"]:
                # 有了圆心和直径, 做视觉伺服对准
                cx, cy = r["circle"].cx_px, r["circle"].cy_px
                diameter = r["diameter_m"]
                self.align_to(cx, cy, diameter)
```

## 依赖

- `opencv-python >= 4.8.0`
- `ultralytics >= 8.0.0`
- `numpy`
- Jetson Orin NX + TensorRT (运行时)

## 参考

- [CUADC/YOLO/infer.py](~/CUADC/YOLO/infer.py) — 已验证的 Jetson 推理模式
- [circle_detect_pipeline.py](src/vision/circle_detect_pipeline.py) — 原始离线圆检测脚本
- [CUADC/数据格式说明.md](~/CUADC/数据格式说明.md) — CSV/时间戳对齐规范
- [针孔相机模型与图像中物体大小计算.md](docs/v2/针孔相机模型与图像中物体大小计算.md)
