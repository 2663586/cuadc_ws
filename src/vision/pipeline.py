#!/usr/bin/env python3
"""
视觉流水线 — YOLO 推理 + 圆检测 + 针孔模型直径计算
=====================================================
将 YOLODetector 与 CircleDetector 串联，提供统一接口供：
  - 实时飞行中逐帧处理 (process_frame)
  - 离线录制视频批量处理 (process_video)

用法:
    from vision import VisionPipeline

    pl = VisionPipeline(model_path="models/yolov11n_800_best_FP16.engine")
    results = pl.process_frame(frame, alt_rel_m=7.0)  # 实时模式

    pl.process_video("video.mp4", "log.csv", "output/")  # 离线模式

参考:
    CUADC/YOLO/infer.py           — YOLO 推理模式
    circle_detect_pipeline.py     — 离线圆检测流程
"""

import csv
import json
import time
import sys
import os
from pathlib import Path
from typing import List, Dict, Optional, Tuple

import cv2
import numpy as np

from config import CIRCLE_CONF_THRESHOLD as _DEFAULT_CIRCLE_CONF

from .yolo_detector import YOLODetector
from .circle_detector import CircleDetector, CircleResult, DEFAULT_CAMERA_MATRIX, DEFAULT_DIST_COEFFS


# ---------------------------------------------------------------------------
# 输出 CSV 列定义
# ---------------------------------------------------------------------------

PIPELINE_CSV_COLUMNS = [
    "frame_idx",
    "monotonic_ns",
    "alt_rel_m",
    "det_idx",
    "conf",
    "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2",
    "edge_success",
    "circle_cx_px", "circle_cy_px",
    "circle_radius_px", "circle_diameter_px",
    "circle_diameter_m",
]


# ---------------------------------------------------------------------------
# 流水线
# ---------------------------------------------------------------------------


class VisionPipeline:
    """YOLO 检测 → 圆检测 → 真实直径 一体的视觉处理流水线"""

    def __init__(
        self,
        model_path: Optional[str] = None,
        camera_matrix: np.ndarray = DEFAULT_CAMERA_MATRIX,
        dist_coeffs: np.ndarray = DEFAULT_DIST_COEFFS,
        bucket_height_m: float = 0.30,
        yolo_conf: float = 0.5,
        yolo_iou: float = 0.45,
        imgsz: int = 800,
        circle_conf_threshold: float = _DEFAULT_CIRCLE_CONF,
        apply_undistort: bool = False,
        max_infer_fps: int = 0,
        **circle_kwargs,
    ):
        """
        Args:
            model_path:            YOLO 模型路径
            camera_matrix:         相机内参 (3×3)
            dist_coeffs:           畸变系数
            bucket_height_m:       桶物理高度 (米)
            yolo_conf:             YOLO 置信度阈值
            yolo_iou:              YOLO NMS IoU 阈值
            imgsz:                 YOLO 输入尺寸
            circle_conf_threshold: 圆检测时使用的最低 YOLO 置信度
            apply_undistort:       是否对每帧做畸变校正
            max_infer_fps:         推理帧率限制 (0=不限)
            **circle_kwargs:       传递给 CircleDetector 的参数
        """
        self.circle_conf_threshold = circle_conf_threshold
        self.apply_undistort = apply_undistort
        self.max_infer_fps = max_infer_fps

        # 子模块
        self.yolo = YOLODetector(
            model_path=model_path,
            imgsz=imgsz,
            conf=yolo_conf,
            iou=yolo_iou,
        )
        self.circle = CircleDetector(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            bucket_height_m=bucket_height_m,
            **circle_kwargs,
        )

        # 畸变校正映射表 (由 process_video 初始化, 或外部调用 init_undistort)
        self._undistort_maps: Optional[Tuple] = None

    # ------------------------------------------------------------------
    # 实时单帧处理
    # ------------------------------------------------------------------

    def process_frame(
        self,
        frame: np.ndarray,
        alt_rel_m: float = 0.0,
        return_annotated: bool = False,
    ) -> List[dict]:
        """
        对单帧执行: YOLO 推理 → 圆检测 → 直径计算。

        Args:
            frame:            BGR 图像 (H×W×3)
            alt_rel_m:        当前相对高度 (米), 用于直径计算
            return_annotated: 是否在返回结果中包含标注帧

        Returns:
            每项 dict:
                {
                    "det": dict,            # YOLO 检测原始数据
                    "circle": CircleResult, # 圆检测结果 (None=未检测到)
                    "diameter_m": float,    # 真实直径 (米)
                    "edge_success": bool,
                }
            无检测时返回空列表。

            若 return_annotated=True, 最后一项为 {"annotated_frame": np.ndarray}
        """
        # 畸变校正
        if self.apply_undistort and self._undistort_maps is not None:
            frame = cv2.remap(
                frame, self._undistort_maps[0], self._undistort_maps[1],
                cv2.INTER_LINEAR,
            )

        # YOLO 推理
        dets = self.yolo.detect(frame)
        bucket_dets = self.yolo.get_class1_detections(
            dets, self.circle_conf_threshold
        )

        results = []
        annotated = frame.copy() if return_annotated else None

        for det in bucket_dets:
            bbox = (det["x1"], det["y1"], det["x2"], det["y2"])

            # 圆检测
            circle_result = self.circle.detect(frame, bbox)

            # 直径计算
            diameter_m = -1.0
            edge_success = False
            if circle_result is not None:
                diameter_m = self.circle.compute_diameter(
                    circle_result.radius_px, alt_rel_m
                )
                edge_success = True
                # 更新 circle_result 中的 diameter_m
                circle_result = CircleResult(
                    cx_px=circle_result.cx_px,
                    cy_px=circle_result.cy_px,
                    radius_px=circle_result.radius_px,
                    diameter_px=circle_result.diameter_px,
                    diameter_m=diameter_m,
                    bbox=circle_result.bbox,
                    roi_offset=circle_result.roi_offset,
                )

            results.append(
                {
                    "det": det,
                    "circle": circle_result,
                    "diameter_m": diameter_m,
                    "edge_success": edge_success,
                }
            )

            # 可视化叠加
            if annotated is not None:
                roi_x1, roi_y1 = (
                    (circle_result.roi_offset)
                    if circle_result
                    else self.circle._extract_roi(frame, *bbox)[1:]
                )
                if circle_result is None:
                    # 无圆检测时也显示边缘
                    roi, rx1, ry1 = self.circle._extract_roi(frame, *bbox)
                    edges = self.circle.get_edges(roi)
                    annotated = self.circle.overlay(
                        annotated, rx1, ry1, edges, None
                    )
                else:
                    roi, rx1, ry1 = self.circle._extract_roi(frame, *bbox)
                    edges = self.circle.get_edges(roi)
                    annotated = self.circle.overlay(
                        annotated, rx1, ry1, edges, circle_result
                    )

        if return_annotated:
            results.append({"annotated_frame": annotated})

        return results

    # ------------------------------------------------------------------
    # 离线批量处理 (视频文件)
    # ------------------------------------------------------------------

    def process_video(
        self,
        video_path: str,
        log_csv_path: str,
        output_dir: str,
        output_video: bool = True,
        frame_limit: int = 0,
    ) -> Path:
        """
        离线处理录制视频: 读取视频 + log.csv, 逐帧 YOLO+圆检测, 输出 CSV + 叠加视频.

        Args:
            video_path:   视频文件路径 (.mp4)
            log_csv_path: 遥测日志路径 (log.csv, 需含 alt_rel_m, monotonic_ns 列)
            output_dir:   输出目录
            output_video: 是否生成带叠加的输出视频
            frame_limit:  最大处理帧数 (0=全部)

        Returns:
            输出的 CSV 文件路径
        """
        video_path = Path(video_path)
        log_csv_path = Path(log_csv_path)
        output_dir = Path(output_dir)

        video_name = video_path.stem
        print(f"\n{'='*60}")
        print(f"  Vision Pipeline: {video_name}")
        print(f"{'='*60}")

        # ---- 1. 加载数据 ----
        print("  [1/4] 加载数据...")

        # 尝试加载 meta.json (可选)
        meta = None
        meta_path = video_path.parent / "meta.json"
        if meta_path.exists():
            with open(meta_path) as f:
                meta = json.load(f)

        # 加载 log.csv
        log_rows = self._load_log_csv(log_csv_path)
        if not log_rows:
            raise RuntimeError(f"无法加载 log.csv: {log_csv_path}")

        # 打开视频
        cap = cv2.VideoCapture(str(video_path))
        if not cap.isOpened():
            raise RuntimeError(f"无法打开视频: {video_path}")

        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        src_fps = cap.get(cv2.CAP_PROP_FPS)

        print(f"        分辨率: {width}×{height} @ {src_fps:.1f}fps")
        print(f"        总帧数: {total_frames}")
        print(f"        log.csv 行数: {len(log_rows)}")

        # ---- 2. 准备输出 ----
        print("  [2/4] 准备输出...")
        output_dir.mkdir(parents=True, exist_ok=True)

        csv_output_path = output_dir / f"{video_name}_circle_detect.csv"
        csv_f = open(csv_output_path, "w", encoding="utf-8-sig", newline="")
        csv_writer = csv.DictWriter(csv_f, fieldnames=PIPELINE_CSV_COLUMNS, extrasaction="ignore")
        csv_writer.writeheader()

        video_writer = None
        if output_video:
            video_output_path = output_dir / f"{video_name}_circle_overlay.mp4"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            video_writer = cv2.VideoWriter(
                str(video_output_path), fourcc, src_fps, (width, height)
            )
            if not video_writer.isOpened():
                print("  ⚠ 视频写入器创建失败, 跳过视频输出")
                video_writer = None

        # ---- 3. 畸变校正初始化 ----
        if self.apply_undistort:
            self._undistort_maps = self.circle.init_undistort_maps(width, height)
            print("        畸变校正: 已启用")
        else:
            print("        畸变校正: 已禁用")

        # ---- 4. 逐帧处理 ----
        print("  [3/4] 逐帧处理...")

        frames_to_process = (
            min(total_frames, frame_limit) if frame_limit > 0 else total_frames
        )
        frame_idx = 0
        circles_found = 0
        csv_row_count = 0
        last_infer_time = 0.0
        start_time = time.time()

        while frame_idx < frames_to_process:
            ret, frame = cap.read()
            if not ret:
                break

            # 帧率限制
            now = time.monotonic()
            if self.max_infer_fps > 0:
                if now - last_infer_time < 1.0 / self.max_infer_fps:
                    # 跳帧
                    if video_writer:
                        video_writer.write(frame)
                    frame_idx += 1
                    continue

            # 遥测数据 (log.csv 按帧 1:1 对应)
            telemetry = (
                log_rows[frame_idx] if frame_idx < len(log_rows) else None
            )
            alt_rel_m = telemetry.get("alt_rel_m", 0.0) if telemetry else 0.0
            mono_ns = telemetry.get("monotonic_ns", 0) if telemetry else 0

            # 推理
            t0 = time.monotonic()
            results = self.process_frame(frame, alt_rel_m=alt_rel_m)
            t1 = time.monotonic()
            last_infer_time = now

            # ---- 写入 CSV ----
            for i, r in enumerate(results):
                det = r["det"]
                circle_r = r["circle"]
                csv_writer.writerow(
                    {
                        "frame_idx": frame_idx,
                        "monotonic_ns": mono_ns,
                        "alt_rel_m": round(alt_rel_m, 6),
                        "det_idx": i,
                        "conf": det["conf"],
                        "bbox_x1": det["x1"],
                        "bbox_y1": det["y1"],
                        "bbox_x2": det["x2"],
                        "bbox_y2": det["y2"],
                        "edge_success": r["edge_success"],
                        "circle_cx_px": (
                            circle_r.cx_px if circle_r else -1
                        ),
                        "circle_cy_px": (
                            circle_r.cy_px if circle_r else -1
                        ),
                        "circle_radius_px": (
                            circle_r.radius_px if circle_r else -1
                        ),
                        "circle_diameter_px": (
                            round(circle_r.diameter_px, 2)
                            if circle_r
                            else -1.0
                        ),
                        "circle_diameter_m": (
                            round(circle_r.diameter_m, 6)
                            if circle_r
                            else -1.0
                        ),
                    }
                )
                csv_row_count += 1
                if r["edge_success"]:
                    circles_found += 1

            # ---- 叠加可视化 ----
            if video_writer:
                annotated_frame = self._build_annotated_frame(frame, results)
                video_writer.write(annotated_frame)

            # 进度
            if frame_idx % 100 == 0 or frame_idx == frames_to_process - 1:
                elapsed = time.time() - start_time
                fps_proc = (
                    (frame_idx + 1) / elapsed if elapsed > 0 else 0
                )
                print(
                    f"\r        帧 {frame_idx + 1}/{frames_to_process} "
                    f"({100 * (frame_idx + 1) / frames_to_process:.1f}%) "
                    f"| {fps_proc:.1f} fps "
                    f"| {circles_found} 个圆已检测",
                    end="",
                    flush=True,
                )

            frame_idx += 1

        # ---- 清理 ----
        cap.release()
        if video_writer:
            video_writer.release()
        csv_f.close()

        elapsed_total = time.time() - start_time
        print(
            f"\n        完成! 耗时 {elapsed_total:.1f}s "
            f"({frame_idx / elapsed_total:.1f} fps)"
        )

        # ---- 统计 ----
        success_ratio = (
            circles_found / csv_row_count * 100 if csv_row_count > 0 else 0
        )
        print(f"\n  📊 统计摘要:")
        print(f"     处理帧数:       {frame_idx}")
        print(f"     CSV 行数:       {csv_row_count}")
        print(f"     圆检测成功:     {circles_found} ({success_ratio:.1f}%)")
        print(f"     输出 CSV:       {csv_output_path}")
        if video_writer:
            print(f"     输出视频:       {output_dir / f'{video_name}_circle_overlay.mp4'}")

        return csv_output_path

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------

    def _build_annotated_frame(
        self, frame: np.ndarray, results: List[dict]
    ) -> np.ndarray:
        """根据 process_frame 结果构建叠加标注帧"""
        annotated = frame.copy()
        for r in results:
            det = r["det"]
            bbox = (det["x1"], det["y1"], det["x2"], det["y2"])
            circle_result = r["circle"]

            roi, rx1, ry1 = self.circle._extract_roi(frame, *bbox)
            edges = self.circle.get_edges(roi)

            if circle_result is not None:
                # 使用 ROI 内坐标进行叠加
                cx_roi = circle_result.cx_px - rx1
                cy_roi = circle_result.cy_px - ry1
                roi_circle = CircleResult(
                    cx_px=cx_roi,
                    cy_px=cy_roi,
                    radius_px=circle_result.radius_px,
                    diameter_px=circle_result.diameter_px,
                    diameter_m=circle_result.diameter_m,
                    bbox=circle_result.bbox,
                    roi_offset=(rx1, ry1),
                )
                annotated = self.circle.overlay(
                    annotated, rx1, ry1, edges, roi_circle
                )
            else:
                annotated = self.circle.overlay(
                    annotated, rx1, ry1, edges, None
                )

        return annotated

    @staticmethod
    def _load_log_csv(log_csv_path: Path) -> List[dict]:
        """加载 log.csv 遥测数据"""
        with open(log_csv_path, "r", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            rows = []
            for r in reader:
                try:
                    r["monotonic_ns"] = int(r["monotonic_ns"])
                    r["alt_rel_m"] = float(r["alt_rel_m"])
                except (KeyError, ValueError):
                    pass
                rows.append(r)
        return rows


# ---------------------------------------------------------------------------
# 命令行入口 (离线处理)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Vision Pipeline — YOLO + Circle Detection"
    )
    parser.add_argument(
        "--video", type=str, required=True, help="输入视频路径"
    )
    parser.add_argument(
        "--log", type=str, required=True, help="log.csv 路径"
    )
    parser.add_argument(
        "--output", type=str, default="output/circle_detect",
        help="输出目录 (默认: output/circle_detect)"
    )
    parser.add_argument(
        "--model", type=str, default=None, help="YOLO 模型路径"
    )
    parser.add_argument(
        "--no-video", action="store_true", help="不生成叠加视频"
    )
    parser.add_argument(
        "--undistort", action="store_true", help="启用畸变校正"
    )
    parser.add_argument(
        "--frames", type=int, default=0, help="最大处理帧数 (0=全部)"
    )
    parser.add_argument(
        "--fps-limit", type=int, default=0, help="推理帧率限制 (0=不限)"
    )

    args = parser.parse_args()

    pl = VisionPipeline(
        model_path=args.model,
        apply_undistort=args.undistort,
        max_infer_fps=args.fps_limit,
    )

    pl.process_video(
        video_path=args.video,
        log_csv_path=args.log,
        output_dir=args.output,
        output_video=not args.no_video,
        frame_limit=args.frames,
    )
