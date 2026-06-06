"""动作空间采样器"""
import numpy as np
from typing import List


def sample_push_actions(state: np.ndarray, config: dict,
                        num_samples: int = 200) -> np.ndarray:
    """
    采样候选推动作。

    Args:
        state: 当前状态 (state_dim,)
        config: 动作空间配置
        num_samples: 采样数量

    Returns:
        actions: (num_samples, action_dim) 候选动作
    """
    action_cfg = config["action_space"]
    n_dirs = action_cfg.get("push_direction_angles", 8)
    d_min = action_cfg.get("push_distance_min", 10)
    d_max = action_cfg.get("push_distance_max", 80)

    actions = np.zeros((num_samples, 4), dtype=np.float32)
    for i in range(num_samples):
        angle = np.random.uniform(0, 2 * np.pi)
        dist = np.random.uniform(d_min, d_max)
        actions[i, 0] = state[0]  # start_x = obj_x
        actions[i, 1] = state[1]  # start_y = obj_y
        actions[i, 2] = angle
        actions[i, 3] = dist
    return actions
