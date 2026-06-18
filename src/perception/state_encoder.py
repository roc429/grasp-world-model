"""×´Ì¬±àÂëÆ÷"""
import numpy as np
from typing import List

def encode_state(objects, state_dim=10, goal_pos=None) -> np.ndarray:
    s = np.zeros(state_dim, dtype=np.float32)
    for obj in objects:
        if obj.label == 'target':
            s[0] = obj.centroid_xy[0]; s[1] = obj.centroid_xy[1]; s[2] = obj.z_height
        elif obj.label == 'obstacle':
            s[3] = obj.centroid_xy[0]; s[4] = obj.centroid_xy[1]; s[5] = obj.z_height
    if goal_pos is not None:
        s[6] = goal_pos[0]; s[7] = goal_pos[1]
    return s
