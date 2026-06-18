"""物体检测 - 从 MuJoCo 状态直接读取位置"""
import numpy as np
from dataclasses import dataclass
from typing import List

@dataclass
class DetectedObject:
    id: int; label: str; centroid_xy: np.ndarray; z_height: float
    bbox: np.ndarray; graspable: bool

def detect_objects(env) -> List[DetectedObject]:
    objs = env.get_objects_state()
    result = []
    if 'target' in objs:
        t = objs['target']
        result.append(DetectedObject(0, 'target', t[:2].copy(), t[2], np.zeros(4), False))
    if 'obstacle' in objs:
        o = objs['obstacle']
        if o[2] > -1:
            result.append(DetectedObject(1, 'obstacle', o[:2].copy(), o[2], np.zeros(4), False))
    return result
