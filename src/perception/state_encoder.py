"""状态编码器 - 将感知结果编码为结构化状态向量"""

from typing import List
import numpy as np
from src.perception.detector import DetectedObject


def encode_state(objects: List[DetectedObject], state_dim: int = 10) -> np.ndarray:
    """
    将检测到的物体列表编码为固定维度状态向量。

    Args:
        objects: 检测到的物体列表
        state_dim: 目标状态维度

    Returns:
        state: 状态向量 (state_dim,)
    """
    state = np.zeros(state_dim, dtype=np.float32)
    # 简单编码: [obj_x, obj_y, obj_z, obstacle_x, obstacle_y, ...]
    idx = 0
    for obj in objects:
        if idx + 3 <= state_dim:
            state[idx] = obj.centroid_xy[0]
            state[idx + 1] = obj.centroid_xy[1]
            state[idx + 2] = obj.z_height
            idx += 3
    return state
