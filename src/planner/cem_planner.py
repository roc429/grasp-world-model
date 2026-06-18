"""CEM (Cross-Entropy Method) 规划器"""
import numpy as np
from typing import Tuple
from src.planner.base_planner import Planner


class CEMPlanner(Planner):
    """CEM 规划器: 迭代优化动作分布"""

    def __init__(self, config: dict):
        super().__init__(config)
        self.num_candidates = config.get("num_candidates", 200)
        self.cem_iterations = config.get("cem_iterations", 5)
        self.elite_ratio = config.get("elite_ratio", 0.2)
        self.action_dim = config.get("action_dim", 4)

    def plan(self, state: np.ndarray,
             world_model: "WorldModel") -> Tuple[np.ndarray, float]:
        n_elite = max(1, int(self.num_candidates * self.elite_ratio))

        # 初始化动作分布 (均值, 标准差)
        mean = np.zeros(self.action_dim, dtype=np.float32)
        mean[0] = state[0]  # start_x
        mean[1] = state[1]  # start_y
        std = np.ones(self.action_dim, dtype=np.float32) * 0.5

        best_action = None
        best_score = -float("inf")

        for iteration in range(self.cem_iterations):
            # 从当前分布采样
            candidates = np.random.randn(self.num_candidates, self.action_dim) * std + mean

            scores = np.zeros(self.num_candidates)
            for i in range(self.num_candidates):
                action_seq = candidates[i].reshape(1, -1)
                _, traj_scores = world_model.predict_trajectory(state, action_seq)
                scores[i] = traj_scores[0]

            # 选精英
            elite_idx = np.argsort(scores)[-n_elite:]
            elite = candidates[elite_idx]

            # 更新分布
            mean = elite.mean(axis=0)
            std = elite.std(axis=0) + 1e-6

            # 跟踪最优
            if scores[elite_idx[-1]] > best_score:
                best_score = scores[elite_idx[-1]]
                best_action = candidates[elite_idx[-1]]

        return best_action, best_score
