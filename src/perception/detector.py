"""物体检测与分割模块"""

from dataclasses import dataclass
from typing import List, Optional
import numpy as np


@dataclass
class DetectedObject:
    """检测到的物体"""
    id: int
    label: str  # "target", "obstacle", "container"
    centroid_xy: np.ndarray  # (2,)
    z_height: float  # mm
    bbox: np.ndarray  # (4,) [x1, y1, x2, y2]
    graspable: bool


class PerceptionError(Exception):
    """感知异常"""
    pass


def detect_objects(
    rgb_image: np.ndarray,
    depth_image: np.ndarray,
    camera_params: dict,
) -> List[DetectedObject]:
    """
    从 RGB-D 图像检测物体。

    Args:
        rgb_image: RGB 图像 (H, W, 3), uint8
        depth_image: 深度图像 (H, W), float32, 单位 mm
        camera_params: 相机内参 {'fx', 'fy', 'cx', 'cy'} + 外参

    Returns:
        detected_objects: 检测到的物体列表

    Raises:
        PerceptionError: 检测失败时抛出
    """
    # TODO: 实现检测逻辑
    raise NotImplementedError("detect_objects not implemented yet")
