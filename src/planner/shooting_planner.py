"""随机 Shooting 规划器"""
import numpy as np
from typing import Tuple
from src.planner.base_planner import Planner
from src.planner.action_sampler import sample_push_actions


class ShootingPlanner(Planner):
    """随机 Shooting: 采样 N 条序列, 选评分最高的"""

    def __init__(self, config: dict):
        super().__init__(config)
        self.num_candidates = config.get("num_candidates", 200)

    def plan(self, state: np.ndarray,
             world_model: "WorldModel") -> Tuple[np.ndarray, float]:
        best_action = None
        best_score = -float("inf")

        actions = sample_push_actions(state, self.config, self.num_candidates)

        for i in range(self.num_candidates):
            action = actions[i]
            action_seq = action.reshape(1, -1)  # 单步, shape (1, action_dim)
            _, scores = world_model.predict_trajectory(state, action_seq)
            score = scores[0]

            if score > best_score:
                best_score = score
                best_action = action

        return best_action, best_score
