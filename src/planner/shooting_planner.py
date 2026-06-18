"""Ëæ»ú Shooting ¹æ»®Æ÷"""
import numpy as np
from src.planner.base_planner import Planner

class ShootingPlanner(Planner):
    def __init__(self, config):
        super().__init__(config)
        self.num_candidates = config.get('num_candidates', 100)

    def plan(self, state, world_model):
        best_action = None
        best_score = -float('inf')
        action_dim = 4
        obj_x, obj_y = state[0], state[1]
        for i in range(self.num_candidates):
            action = np.array([
                obj_x + np.random.uniform(-0.02, 0.02),
                obj_y + np.random.uniform(-0.02, 0.02),
                np.random.uniform(0, 2 * np.pi),
                np.random.uniform(0.01, 0.08)
            ], dtype=np.float32)
            _, scores = world_model.predict_trajectory(state, action.reshape(1, -1))
            score = float(scores[0])
            if score > best_score:
                best_score = score
                best_action = action.copy()
        return best_action, best_score
