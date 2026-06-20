"""状态编码器 - 将感知结果编码为结构化状态向量

编码约定 (state_dim=10) - 与 SimulationEnv._build_state() 格式一致:
  [0:3]   target (x, y, z)       - 目标物体位置 (mm)
  [3:6]   obstacle (x, y, z)     - 障碍物位置 (mm)
  [6:9]   ee (x, y, z)           - 末端执行器位置 (mm)
  [9]     gripper_state          - 夹爪状态 (1.0=open, 0.0=closed)
"""

from typing import List, Dict
import numpy as np
from src.perception.detector import DetectedObject


SLOT_MAP: Dict[str, int] = {
    "target":    0,
    "obstacle":  3,
    "container": 3,
}
OBJECT_SLOT_SIZE = 3


def encode_state(
    objects: List[DetectedObject],
    state_dim: int = 10,
    ee_xyz: np.ndarray = None,
    gripper_open: bool = True,
) -> np.ndarray:
    """
    将检测到的物体列表编码为固定维度状态向量。

    按 label 分配到固定槽位，保证世界模型输入的语义一致性。
    格式与 SimulationEnv._build_state() 完全对齐。

    Args:
        objects: 检测到的物体列表
        state_dim: 目标状态维度 (默认 10)
        ee_xyz: 末端执行器位置 (3,) mm，默认原点
        gripper_open: 夹爪是否打开

    Returns:
        state: 状态向量 (state_dim,)
    """
    state = np.zeros(state_dim, dtype=np.float32)

    for obj in objects:
        slot_start = SLOT_MAP.get(obj.label)
        if slot_start is None:
            continue
        end = slot_start + OBJECT_SLOT_SIZE
        if end > state_dim:
            continue
        state[slot_start]     = obj.centroid_xy[0]
        state[slot_start + 1] = obj.centroid_xy[1]
        state[slot_start + 2] = obj.z_height

    if ee_xyz is not None and 9 <= state_dim:
        state[6:9] = ee_xyz
    if 9 < state_dim:
        state[9] = 1.0 if gripper_open else 0.0

    return state


def decode_state(state: np.ndarray) -> dict:
    """将状态向量解码为可读字典"""
    return {
        "target_xy":    state[0:2].copy(),
        "target_z":     float(state[2]),
        "obstacle_xy":  state[3:5].copy(),
        "obstacle_z":   float(state[5]),
        "ee_xyz":       state[6:9].copy(),
        "gripper_open": bool(state[9] > 0.5) if len(state) > 9 else False,
    }
