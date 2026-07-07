#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
桶形目标圆检测脚本 (Circle Detection Pipeline)
================================================
功能：
  1. 读取 YOLO infer.csv 中 class=1 (bucket) 的检测框
  2. 在每个检测框 ROI 内进行 Canny 边缘检测 + HoughCircles 圆检测
  3. 基于针孔相机模型，结合 log.csv 中的 alt_rel_m 高度数据，计算圆的真实直径
  4. 输出 CSV 日志（圆心像素坐标、像素直径、估算真实直径等）
  5. 生成叠加了 Canny 边缘 + 检测圆 + 圆心的输出视频

用法：
  conda activate OpenCVTest
  python d:/Code/VsCode/cvtest/circle_detect_pipeline.py

依赖：
  - opencv-python (cv2)
  - numpy
  - 标准库: csv, json, bisect, pathlib, time, sys

⚠️ 相机内参 (CAMERA_MATRIX) 当前使用估算值 fx=fy=800, cx=640, cy=360，
   实际使用时需替换为标定得到的真实内参矩阵。
   标定方法参见: 针孔相机模型与图像中物体大小计算.md
"""

import cv2
import numpy as np
import csv
import json
import bisect
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any
import sys
import time

# ============================================================
# 配置区 — 按需修改
# ============================================================

# 待处理的视频目录（只需处理有 infer.csv 的视频）
VIDEO_DIRS = [
    Path("d:/Code/VsCode/cvtest/video/20260701_153306"),
    Path("d:/Code/VsCode/cvtest/video/20260701_153759"),
]

# 输出根目录
OUTPUT_DIR = Path("d:/Code/VsCode/cvtest/circle_detect_output")

# ============================================================
# 相机内参 — 由棋盘格标定获得 (平均重投影误差 0.1784 px)
# 摄像头: Orin NX 下视, 1280×720
# ============================================================
CAMERA_MATRIX = np.array([
    [1059.1542,    0.0000,  638.3737],
    [   0.0000, 1058.8622,  355.4191],
    [   0.0000,    0.0000,    1.0000]
], dtype=np.float64)

# 畸变系数 (k1, k2, p1, p2, k3)
DIST_COEFFS = np.array(
    [0.034250, -0.037603, 0.003390, 0.002477, -0.017466],
    dtype=np.float64
)

# 是否对每一帧做畸变校正（True=更精确但稍慢, False=跳过）
APPLY_UNDISTORT = True

# 从内参矩阵提取各参数
FX = float(CAMERA_MATRIX[0, 0])  # 焦距 x (pixel) = 1059.15
FY = float(CAMERA_MATRIX[1, 1])  # 焦距 y (pixel) = 1058.86
CX = float(CAMERA_MATRIX[0, 2])  # 主点 x = 638.37
CY = float(CAMERA_MATRIX[1, 2])  # 主点 y = 355.42

# 桶的物理高度（米），用于修正相机到桶顶的距离
# Z_C = alt_rel_m - BUCKET_HEIGHT_M
BUCKET_HEIGHT_M = 0.30

# YOLO 检测置信度阈值（低于此值的检测框被忽略）
CONFIDENCE_THRESHOLD = 0.3

# ROI 扩展比例（在 YOLO bbox 基础上向外扩展，为边缘检测提供上下文）
ROI_PADDING_RATIO = 0.20

# ---- Canny 边缘检测参数 ----
CANNY_LOW_THRESHOLD = 40
CANNY_HIGH_THRESHOLD = 120

# ---- HoughCircles 圆检测参数 ----
HOUGH_DP = 1.2           # 累加器分辨率与图像分辨率的反比
HOUGH_MIN_DIST = 20      # 检测到的圆之间的最小距离（像素）
HOUGH_PARAM1 = 50        # Canny 高阈值（HoughCircles 内部 Canny）
HOUGH_PARAM2 = 25        # 累加器阈值（越小检测到越多假圆）
HOUGH_MIN_RADIUS = 5     # 最小圆半径（像素）
HOUGH_MAX_RADIUS = 100   # 最大圆半径（像素）

# ---- 可视化参数 ----
EDGE_COLOR = (0, 255, 0)        # Canny 边缘颜色（绿色）
CIRCLE_COLOR = (0, 0, 255)      # 检测圆颜色（红色）
CENTER_COLOR = (0, 0, 255)      # 圆心颜色（红色）
TEXT_COLOR = (255, 255, 255)    # 文字颜色（白色）
OVERLAY_ALPHA = 0.5             # 边缘叠加透明度

# ---- 视频输出参数 ----
OUTPUT_FPS = 30  # 输出视频帧率（与原始一致）
# Windows 上优先尝试 avc1 (H.264)，失败则回退到 mp4v
VIDEO_CODECS = ['avc1', 'mp4v']

# ---- 处理控制 ----
MAX_FRAMES = 0  # 最大处理帧数，0 = 不限制（用于调试）


# ============================================================
# 辅助函数
# ============================================================

def load_meta(rec_dir: Path) -> dict:
    """加载 meta.json"""
    with open(rec_dir / "meta.json", "r") as f:
        return json.load(f)


def load_log_csv(rec_dir: Path) -> List[dict]:
    """
    加载 log.csv（飞控遥测），返回按行序排列的字典列表。
    numeric 字段自动转为 float/int。
    """
    with open(rec_dir / "log.csv", "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = []
        for r in reader:
            r["monotonic_ns"] = int(r["monotonic_ns"])
            r["alt_rel_m"] = float(r["alt_rel_m"])
            r["lat_deg"] = float(r["lat_deg"])
            r["lon_deg"] = float(r["lon_deg"])
            r["roll_deg"] = float(r["roll_deg"])
            r["pitch_deg"] = float(r["pitch_deg"])
            r["yaw_deg"] = float(r["yaw_deg"])
            r["heading_deg"] = float(r["heading_deg"])
            r["groundspeed_mps"] = float(r["groundspeed_mps"])
            r["hdop"] = float(r["hdop"])
            r["vdop"] = float(r["vdop"])
            rows.append(r)
    return rows


def load_infer_csv(rec_dir: Path) -> List[dict]:
    """
    加载 infer.csv（YOLO 检测结果），返回字典列表。
    numeric 字段自动转为 int/float。
    """
    video_name = rec_dir.name
    infer_path = rec_dir / f"{video_name}_infer.csv"
    if not infer_path.exists():
        print(f"  ⚠ 未找到 infer.csv: {infer_path}")
        return []

    with open(infer_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = []
        for r in reader:
            r["monotonic_ns"] = int(r["monotonic_ns"])
            r["frame_idx"] = int(r["frame_idx"])
            r["det_idx"] = int(r["det_idx"])
            r["cls"] = int(r["cls"])
            r["conf"] = float(r["conf"])
            r["u_px"] = int(r["u_px"])
            r["v_px"] = int(r["v_px"])
            r["w_px"] = int(r["w_px"])
            r["h_px"] = int(r["h_px"])
            r["x1"] = int(r["x1"])
            r["y1"] = int(r["y1"])
            r["x2"] = int(r["x2"])
            r["y2"] = int(r["y2"])
            rows.append(r)
    return rows


def get_class1_detections_for_frame(infer_rows: List[dict], frame_idx: int,
                                    conf_threshold: float) -> List[dict]:
    """
    获取指定帧中所有 class=1 (bucket) 且置信度 ≥ 阈值的检测。
    返回按置信度降序排列。
    """
    dets = [d for d in infer_rows
            if d["frame_idx"] == frame_idx
            and d["cls"] == 1
            and d["conf"] >= conf_threshold]
    dets.sort(key=lambda d: d["conf"], reverse=True)
    return dets


def extract_roi(frame: np.ndarray, x1: int, y1: int, x2: int, y2: int,
                padding_ratio: float) -> Tuple[np.ndarray, int, int]:
    """
    从帧中提取 ROI（带外扩 padding），返回 (roi, roi_x1, roi_y1)。
    roi_x1/roi_y1 是 ROI 在原始帧中的左上角坐标，用于坐标回算。
    """
    h, w = frame.shape[:2]
    bw, bh = x2 - x1, y2 - y1
    pad_w = int(bw * padding_ratio)
    pad_h = int(bh * padding_ratio)

    rx1 = max(0, x1 - pad_w)
    ry1 = max(0, y1 - pad_h)
    rx2 = min(w, x2 + pad_w)
    ry2 = min(h, y2 + pad_h)

    # 确保 ROI 有效
    if rx2 <= rx1 or ry2 <= ry1:
        rx1, ry1, rx2, ry2 = x1, y1, x2, y2

    roi = frame[ry1:ry2, rx1:rx2]
    return roi, rx1, ry1


def detect_circle_in_roi(roi: np.ndarray) -> Optional[Tuple[int, int, int]]:
    """
    在 ROI 灰度图中检测圆。

    流程：灰度化 → 高斯模糊 → Canny 边缘检测 → HoughCircles

    Returns:
        (cx_roi, cy_roi, radius) — ROI 内的圆心坐标和半径（像素）
        如果未检测到圆，返回 None
    """
    if roi.size == 0 or roi.shape[0] < 10 or roi.shape[1] < 10:
        return None

    # 灰度化
    if len(roi.shape) == 3:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        gray = roi.copy()

    # 高斯模糊降噪
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 限制 HoughCircles 的半径范围不超过 ROI 尺寸的一半
    min_r = max(HOUGH_MIN_RADIUS, 3)
    max_r = min(HOUGH_MAX_RADIUS, min(roi.shape[0], roi.shape[1]) // 2)

    if max_r < min_r:
        return None

    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=HOUGH_DP,
        minDist=HOUGH_MIN_DIST,
        param1=HOUGH_PARAM1,
        param2=HOUGH_PARAM2,
        minRadius=min_r,
        maxRadius=max_r
    )

    if circles is None or len(circles[0]) == 0:
        return None

    # 选择半径最大的圆（通常最可靠）
    circles = np.uint16(np.around(circles[0]))
    best = max(circles, key=lambda c: c[2])
    cx_roi, cy_roi, radius = int(best[0]), int(best[1]), int(best[2])

    return cx_roi, cy_roi, radius


def compute_circle_diameter_real(radius_px: int, alt_rel_m: float) -> float:
    """
    基于针孔相机模型计算圆的真实直径。

    公式：D_real = (2 * r_px * Z_C) / fx

    其中：
      Z_C = alt_rel_m - BUCKET_HEIGHT_M  (相机到桶顶的距离)
      r_px = 圆的像素半径
      fx   = 相机焦距 (像素)

    Returns:
        真实直径（米），如果 alt_rel_m ≤ bucket_height 则返回 -1
    """
    z_c = alt_rel_m - BUCKET_HEIGHT_M
    if z_c <= 0.0:
        return -1.0  # 高度无效
    diameter_px = 2.0 * radius_px
    diameter_m = (diameter_px * z_c) / FX
    return diameter_m


def get_canny_edges(roi: np.ndarray) -> np.ndarray:
    """
    对 ROI 执行 Canny 边缘检测，返回二值边缘图。
    """
    if len(roi.shape) == 3:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        gray = roi.copy()

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)
    return edges


def overlay_edges_and_circle(frame: np.ndarray,
                             roi_x1: int, roi_y1: int,
                             edges: np.ndarray,
                             circle: Optional[Tuple[int, int, int]],
                             diameter_m: float) -> np.ndarray:
    """
    在帧上叠加 Canny 边缘（半透明绿色）和检测圆/圆心（红色）。

    Args:
        frame: 原始帧 (BGR)
        roi_x1, roi_y1: ROI 在全帧中的左上角坐标
        edges: Canny 边缘二值图 (与 ROI 同尺寸)
        circle: (cx_roi, cy_roi, radius) 或 None
        diameter_m: 真实直径（米），用于标注文字

    Returns:
        叠加后的帧
    """
    result = frame.copy()
    h_roi, w_roi = edges.shape[:2]
    roi_y2 = roi_y1 + h_roi
    roi_x2 = roi_x1 + w_roi

    # 确保 ROI 区域不越界
    if roi_y2 > result.shape[0]:
        roi_y2 = result.shape[0]
        h_roi = roi_y2 - roi_y1
    if roi_x2 > result.shape[1]:
        roi_x2 = result.shape[1]
        w_roi = roi_x2 - roi_x1

    if h_roi <= 0 or w_roi <= 0:
        return result

    # 裁剪 edges 以匹配有效区域
    edges_cropped = edges[:h_roi, :w_roi]

    # 半透明绿色叠加 Canny 边缘
    roi_region = result[roi_y1:roi_y2, roi_x1:roi_x2]
    green_overlay = np.zeros_like(roi_region)
    green_overlay[edges_cropped > 0] = EDGE_COLOR
    blended = cv2.addWeighted(roi_region, 1.0 - OVERLAY_ALPHA,
                              green_overlay, OVERLAY_ALPHA, 0)
    result[roi_y1:roi_y2, roi_x1:roi_x2] = blended

    # 绘制检测圆和圆心
    if circle is not None:
        cx_roi, cy_roi, radius = circle
        # 转换到全帧坐标
        cx_full = roi_x1 + cx_roi
        cy_full = roi_y1 + cy_roi

        # 确保圆心在图像范围内
        cx_full = np.clip(cx_full, 0, result.shape[1] - 1)
        cy_full = np.clip(cy_full, 0, result.shape[0] - 1)

        # 画圆（红色，线宽 2）
        cv2.circle(result, (cx_full, cy_full), radius, CIRCLE_COLOR, 2)
        # 画圆心（红色实心点）
        cv2.circle(result, (cx_full, cy_full), 3, CENTER_COLOR, -1)
        # 画十字准线
        cross_size = radius // 2 if radius > 6 else 3
        cv2.line(result, (cx_full - cross_size, cy_full),
                 (cx_full + cross_size, cy_full), CENTER_COLOR, 1)
        cv2.line(result, (cx_full, cy_full - cross_size),
                 (cx_full, cy_full + cross_size), CENTER_COLOR, 1)

        # 标注直径
        if diameter_m > 0:
            label = f"D={diameter_m*100:.1f}cm"
        else:
            label = f"r={radius}px"
        cv2.putText(result, label, (cx_full + radius + 5, cy_full - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_COLOR, 1)

    return result


def build_infer_index(infer_rows: List[dict]) -> Dict[int, List[dict]]:
    """
    将 infer_rows 按 frame_idx 建立索引，加速按帧查找。
    """
    index: Dict[int, List[dict]] = {}
    for d in infer_rows:
        fi = d["frame_idx"]
        if fi not in index:
            index[fi] = []
        index[fi].append(d)
    return index


# ============================================================
# 主处理流程（单个视频）
# ============================================================

def process_video(rec_dir: Path) -> Optional[Path]:
    """
    处理单个视频目录：读取数据 → 逐帧处理 → 输出 CSV + 叠加视频。

    Returns:
        输出的 CSV 文件路径，失败返回 None
    """
    video_name = rec_dir.name
    print(f"\n{'='*60}")
    print(f"  处理视频: {video_name}")
    print(f"{'='*60}")

    # ---- 1. 加载数据 ----
    print("  [1/5] 加载数据...")
    meta = load_meta(rec_dir)
    log_rows = load_log_csv(rec_dir)
    infer_rows = load_infer_csv(rec_dir)

    video_path = rec_dir / f"{video_name}.mp4"
    if not video_path.exists():
        print(f"  ❌ 视频文件不存在: {video_path}")
        return None

    total_frames = meta["total_frames"]
    width = meta["width"]
    height = meta["height"]
    fps = meta["fps"]

    print(f"        分辨率: {width}×{height} @ {fps}fps")
    print(f"        总帧数: {total_frames}")
    print(f"        log.csv 行数: {len(log_rows)}")
    print(f"        infer.csv class=1 检测数: "
          f"{sum(1 for d in infer_rows if int(d['cls']) == 1)}")

    # 建立 infer 按帧索引
    infer_index = build_infer_index(infer_rows)

    # ---- 2. 准备输出 ----
    print("  [2/5] 准备输出文件...")
    video_output_dir = OUTPUT_DIR / video_name
    video_output_dir.mkdir(parents=True, exist_ok=True)

    csv_output_path = video_output_dir / f"{video_name}_circle_detect.csv"
    video_output_path = video_output_dir / f"{video_name}_circle_overlay.mp4"

    # ---- 3. 打开视频 ----
    print("  [3/5] 打开视频文件...")
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"  ❌ 无法打开视频: {video_path}")
        return None

    # ---- 4. 初始化输出视频写入器 ----
    print("  [4/5] 初始化视频编码器...")
    fourcc = None
    writer = None
    for codec_name in VIDEO_CODECS:
        fourcc = cv2.VideoWriter_fourcc(*codec_name)
        test_writer = cv2.VideoWriter(
            str(video_output_path), fourcc, OUTPUT_FPS, (width, height)
        )
        if test_writer.isOpened():
            writer = test_writer
            print(f"        使用编码器: {codec_name}")
            break
        test_writer.release()

    if writer is None or not writer.isOpened():
        print(f"  ❌ 无法创建视频写入器，尝试过的编码器: {VIDEO_CODECS}")
        cap.release()
        return None

    # ---- 5. 逐帧处理 ----
    print("  [5/5] 逐帧处理...")

    # 预计算畸变校正映射表（耗时一次, 后续 remap 很快）
    if APPLY_UNDISTORT:
        map1, map2 = cv2.initUndistortRectifyMap(
            CAMERA_MATRIX, DIST_COEFFS, None, CAMERA_MATRIX,
            (width, height), cv2.CV_16SC2
        )
        print("        畸变校正: 已启用")
    else:
        print("        畸变校正: 已禁用")

    csv_rows = []
    frame_idx = 0
    frames_processed = 0
    frames_with_detection = 0
    circles_found = 0
    start_time = time.time()
    frames_to_process = min(total_frames, MAX_FRAMES) if MAX_FRAMES > 0 else total_frames

    while frame_idx < frames_to_process:
        ret, frame = cap.read()
        if not ret:
            break

        # 畸变校正
        if APPLY_UNDISTORT:
            frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

        # 获取当前帧的遥测数据（log.csv 按帧 1:1 对应）
        telemetry = log_rows[frame_idx] if frame_idx < len(log_rows) else None
        alt_rel_m = telemetry["alt_rel_m"] if telemetry else 0.0

        # 获取当前帧的 class=1 检测
        frame_dets = infer_index.get(frame_idx, [])
        class1_dets = [d for d in frame_dets
                       if d["cls"] == 1 and d["conf"] >= CONFIDENCE_THRESHOLD]

        # 处理每个 class=1 检测框
        for det in class1_dets:
            x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]

            # 提取 ROI
            roi, roi_x1, roi_y1 = extract_roi(
                frame, x1, y1, x2, y2, ROI_PADDING_RATIO
            )

            # Canny 边缘检测
            edges = get_canny_edges(roi)

            # HoughCircles 圆检测
            circle_result = detect_circle_in_roi(roi)

            # 计算直径
            diameter_m = -1.0
            circle_cx_px = -1
            circle_cy_px = -1
            circle_radius_px = -1
            circle_diameter_px = -1.0
            edge_success = False

            if circle_result is not None:
                circles_found += 1
                cx_roi, cy_roi, radius = circle_result
                circle_cx_px = roi_x1 + cx_roi
                circle_cy_px = roi_y1 + cy_roi
                circle_radius_px = radius
                circle_diameter_px = 2.0 * radius
                diameter_m = compute_circle_diameter_real(radius, alt_rel_m)
                edge_success = True

                # 在帧上叠加边缘和圆
                frame = overlay_edges_and_circle(
                    frame, roi_x1, roi_y1, edges,
                    (cx_roi, cy_roi, radius), diameter_m
                )
            else:
                # 即使未检测到圆，也叠加 Canny 边缘
                frame = overlay_edges_and_circle(
                    frame, roi_x1, roi_y1, edges, None, -1.0
                )

            # 记录 CSV 行
            csv_rows.append({
                "video_name": video_name,
                "frame_idx": frame_idx,
                "monotonic_ns": det["monotonic_ns"],
                "alt_rel_m": round(alt_rel_m, 6),
                "det_idx": det["det_idx"],
                "conf": det["conf"],
                "bbox_x1": x1,
                "bbox_y1": y1,
                "bbox_x2": x2,
                "bbox_y2": y2,
                "edge_success": edge_success,
                "circle_cx_px": circle_cx_px,
                "circle_cy_px": circle_cy_px,
                "circle_radius_px": circle_radius_px,
                "circle_diameter_px": round(circle_diameter_px, 2),
                "circle_diameter_m": round(diameter_m, 6),
            })

        if class1_dets:
            frames_with_detection += 1

        # 写入输出视频
        writer.write(frame)
        frames_processed += 1

        # 进度报告（每 100 帧或最后一帧）
        if frame_idx % 100 == 0 or frame_idx == frames_to_process - 1:
            elapsed = time.time() - start_time
            fps_proc = (frame_idx + 1) / elapsed if elapsed > 0 else 0
            print(f"\r        帧 {frame_idx + 1}/{frames_to_process} "
                  f"({100*(frame_idx+1)/frames_to_process:.1f}%) "
                  f"| {fps_proc:.1f} fps "
                  f"| {circles_found} 个圆已检测", end="", flush=True)

        frame_idx += 1

    # 释放资源
    cap.release()
    writer.release()

    elapsed_total = time.time() - start_time
    print(f"\n        完成! 耗时 {elapsed_total:.1f}s "
          f"({frames_processed/elapsed_total:.1f} fps)")

    # ---- 6. 写入 CSV ----
    print("  [6/6] 写入 CSV 日志...")
    if csv_rows:
        fieldnames = [
            "video_name", "frame_idx", "monotonic_ns", "alt_rel_m",
            "det_idx", "conf",
            "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2",
            "edge_success",
            "circle_cx_px", "circle_cy_px",
            "circle_radius_px", "circle_diameter_px",
            "circle_diameter_m",
        ]
        with open(csv_output_path, "w", encoding="utf-8-sig", newline="") as f:
            writer_csv = csv.DictWriter(f, fieldnames=fieldnames)
            writer_csv.writeheader()
            writer_csv.writerows(csv_rows)

        print(f"        CSV 已保存: {csv_output_path}")
        print(f"        共 {len(csv_rows)} 行 ({circles_found} 个圆检测成功)")

    # ---- 统计摘要 ----
    success_ratio = circles_found / len(csv_rows) * 100 if csv_rows else 0
    print(f"\n  📊 统计摘要:")
    print(f"     处理帧数:       {frames_processed}")
    print(f"     有检测的帧数:   {frames_with_detection}")
    print(f"     CSV 行数:       {len(csv_rows)}")
    print(f"     圆检测成功:     {circles_found} ({success_ratio:.1f}%)")
    print(f"     输出视频:       {video_output_path}")
    print(f"     输出 CSV:       {csv_output_path}")

    return csv_output_path


# ============================================================
# 主入口
# ============================================================

def main():
    print("=" * 60)
    print("  桶形目标圆检测管线 (Circle Detection Pipeline)")
    print("=" * 60)
    print(f"  相机内参 fx={FX:.1f}, fy={FY:.1f}, cx={CX:.1f}, cy={CY:.1f}")
    print(f"  桶高度: {BUCKET_HEIGHT_M}m")
    print(f"  置信度阈值: {CONFIDENCE_THRESHOLD}")
    print(f"  检测模式: Canny + HoughCircles")
    print(f"  针孔模型: D_real = (D_px × Z_C) / fx")
    print()

    all_ok = True
    for rec_dir in VIDEO_DIRS:
        if not rec_dir.exists():
            print(f"  ⚠ 目录不存在，跳过: {rec_dir}")
            all_ok = False
            continue

        # 检查必要文件
        required = ["meta.json", "log.csv", f"{rec_dir.name}.mp4"]
        missing = [f for f in required if not (rec_dir / f).exists()]
        if missing:
            print(f"  ⚠ {rec_dir.name}: 缺少文件 {missing}，跳过")
            all_ok = False
            continue

        try:
            process_video(rec_dir)
        except Exception as e:
            print(f"  ❌ {rec_dir.name} 处理失败: {e}")
            import traceback
            traceback.print_exc()
            all_ok = False

    print(f"\n{'='*60}")
    if all_ok:
        print("  ✅ 全部处理完成")
    else:
        print("  ⚠ 部分视频处理失败，请检查上方日志")
    print(f"  输出目录: {OUTPUT_DIR}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
