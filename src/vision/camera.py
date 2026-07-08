"""
共享相机模块 — 惰性初始化 + 线程池异步帧捕获。

用法:
    from vision.camera import capture_frame_async

    frame = await capture_frame_async()  # → np.ndarray (BGR, H×W×3)

环境变量:
    CAMERA_DEVICE   设备路径/索引 (默认 0，即 /dev/video0)
    CAMERA_WIDTH    分辨率宽度 (默认 1280)
    CAMERA_HEIGHT   分辨率高度 (默认 720)
"""

import asyncio
import concurrent.futures
import os
from typing import Optional

import cv2
import numpy as np

_camera: Optional[cv2.VideoCapture] = None


def _get_camera() -> cv2.VideoCapture:
    """惰性初始化和返回全局相机实例。"""
    global _camera

    if _camera is not None and _camera.isOpened():
        return _camera

    device = os.environ.get("CAMERA_DEVICE", "0")
    try:
        device_id = int(device)
        _camera = cv2.VideoCapture(device_id)
    except ValueError:
        _camera = cv2.VideoCapture(device)

    if not _camera.isOpened():
        raise RuntimeError(f"无法打开摄像头: {device}")

    width = int(os.environ.get("CAMERA_WIDTH", "1280"))
    height = int(os.environ.get("CAMERA_HEIGHT", "720"))
    _camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    _camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    return _camera


async def capture_frame_async() -> np.ndarray:
    """在线程池中异步捕获一帧 BGR 图像。"""
    cam = _get_camera()
    loop = asyncio.get_running_loop()
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
        ret, frame = await loop.run_in_executor(executor, cam.read)
    if not ret:
        raise RuntimeError("相机帧捕获失败")
    return frame


def release_camera():
    """释放相机资源。"""
    global _camera
    if _camera is not None:
        _camera.release()
        _camera = None
