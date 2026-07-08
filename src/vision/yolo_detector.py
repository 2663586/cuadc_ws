#!/usr/bin/env python3
"""
YOLO 检测器模块 — Jetson Orin NX (TensorRT)
=============================================
封装 Ultralytics YOLO 模型加载与推理，供实时/离线视觉流水线调用。

用法:
    from vision.yolo_detector import YOLODetector

    detector = YOLODetector("models/yolov11n_800_best_FP16.engine")
    dets = detector.detect(frame)  # → [{x1,y1,x2,y2,conf,cls,name}, ...]

模型:
    默认使用 TensorRT FP16 engine (imgsz=800), 检测 2 类:
      cls=0: blue_background (蓝色背景板)
      cls=1: bucket (桶)

参考:
    CUADC/YOLO/infer.py — 已验证的 Jetson Orin NX 推理模式
"""

import os
import numpy as np
from pathlib import Path
from typing import List, Dict, Optional


# 默认模型路径 — 相对于本文件所在目录的 models/
_DEFAULT_MODEL_NAME = "yolov11n_800_best_FP16.engine"
_DEFAULT_MODEL_DIR = Path(__file__).resolve().parent / "models"


class YOLODetector:
    """YOLO 目标检测器 (TensorRT / PyTorch)"""

    def __init__(
        self,
        model_path: Optional[str] = None,
        imgsz: int = 800,
        conf: float = 0.5,
        iou: float = 0.45,
    ):
        """
        Args:
            model_path: 模型文件路径 (.engine / .pt / .onnx).
                        默认: src/vision/models/yolov11n_800_best_FP16.engine
            imgsz:     模型输入尺寸 (默认 800)
            conf:      置信度阈值 (0~1)
            iou:       NMS IoU 阈值
        """
        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        self.model = None
        self._names: Dict[int, str] = {}

        # 解析模型路径
        if model_path is None:
            model_path = str(_DEFAULT_MODEL_DIR / _DEFAULT_MODEL_NAME)
        self.model_path = self._resolve_model_path(model_path)
        self._load()

    # ------------------------------------------------------------------
    # 模型加载
    # ------------------------------------------------------------------

    def _resolve_model_path(self, path: str) -> str:
        """尝试自动补全模型文件扩展名 (.engine → .pt → .onnx)"""
        if os.path.exists(path):
            return path

        base = path
        for ext in [".engine", ".pt", ".onnx"]:
            # 去除已有扩展名再试
            for old_ext in [".engine", ".pt", ".onnx"]:
                if base.endswith(old_ext):
                    base = base[: -len(old_ext)]
                    break
            candidate = base + ext
            if os.path.exists(candidate):
                return candidate

        # 都找不到, 返回原始路径 (加载时会报错)
        return path

    def _load(self):
        """加载模型"""
        mp = self.model_path
        if not os.path.exists(mp):
            raise FileNotFoundError(
                f"模型文件未找到: {mp}\n"
                f"请将 TensorRT engine 放到 src/vision/models/ 目录下"
            )

        try:
            from ultralytics import YOLO

            self.model = YOLO(mp)
            backend = "TensorRT" if mp.endswith(".engine") else "PyTorch"
            if hasattr(self.model, "names"):
                self._names = self.model.names
            print(f"[YOLODetector] 模型已加载: {mp} [{backend}]")
        except Exception as e:
            raise RuntimeError(f"模型加载失败 ({mp}): {e}") from e

    # ------------------------------------------------------------------
    # 推理
    # ------------------------------------------------------------------

    def detect(self, frame: np.ndarray) -> List[dict]:
        """
        对单帧图像进行目标检测。

        Args:
            frame: BGR 图像 (numpy ndarray, H×W×3)

        Returns:
            检测结果列表, 每项为:
                {
                    "x1": int, "y1": int, "x2": int, "y2": int,  # 边界框 (像素坐标)
                    "conf": float,                                  # 置信度 (0~1)
                    "cls": int,                                     # 类别 ID
                    "name": str,                                    # 类别名称
                }
            无检测时返回空列表。
        """
        if self.model is None:
            return []

        results = self.model(
            frame,
            verbose=False,
            device=0,
            imgsz=self.imgsz,
            conf=self.conf,
            iou=self.iou,
            half=True,  # FP16
        )

        dets = []
        for r in results:
            boxes = r.boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cls_id = int(box.cls[0])
                    dets.append(
                        {
                            "x1": int(x1),
                            "y1": int(y1),
                            "x2": int(x2),
                            "y2": int(y2),
                            "conf": round(float(box.conf[0]), 3),
                            "cls": cls_id,
                            "name": self._names.get(cls_id, "?"),
                        }
                    )
        return dets

    # ------------------------------------------------------------------
    # 属性
    # ------------------------------------------------------------------

    @property
    def names(self) -> Dict[int, str]:
        """类别名称映射 {cls_id: name}"""
        return self._names

    def get_class1_detections(
        self, dets: List[dict], conf_threshold: Optional[float] = None
    ) -> List[dict]:
        """
        过滤出 class=1 (bucket) 且置信度达标的检测, 按置信度降序排列。

        Args:
            dets:          detect() 返回的检测列表
            conf_threshold: 置信度阈值, 默认使用实例的 self.conf

        Returns:
            过滤并排序后的 bucket 检测列表
        """
        threshold = conf_threshold if conf_threshold is not None else self.conf
        bucket_dets = [
            d for d in dets if d["cls"] == 1 and d["conf"] >= threshold
        ]
        bucket_dets.sort(key=lambda d: d["conf"], reverse=True)
        return bucket_dets
