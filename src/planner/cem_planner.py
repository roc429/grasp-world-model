"""CEM ¹æ»®Æ÷"""
import numpy as np
from src.planner.base_planner import Planner

class CEMPlanner(Planner):
    def __init__(self, config):
        super().__init__(config)
        self.num_candidates = config.get('num_candidates', 100)
        self.cem_iters = config.get('cem_iterations', 4)
        self.elite_ratio = config.get('elite_ratio', 0.2)
        self.ad = 4

    def plan(self, state, world_model):
        n_elite = max(1, int(self.num_candidates * self.elite_ratio))
        mean = np.array([state[0], state[1], 0.0, 0.04], dtype=np.float32)
        std = np.array([0.03, 0.03, 3.14, 0.04], dtype=np.float32)
        best_a, best_s = None, -float('inf')
        for _ in range(self.cem_iters):
            cand = np.random.randn(self.num_candidates, self.ad).astype(np.float32) * std + mean
            scores = np.zeros(self.num_candidates, dtype=np.float32)
            for i in range(self.num_candidates):
                _, ss = world_model.predict_trajectory(state, cand[i].reshape(1, -1))
                scores[i] = float(ss[0])
            elite_idx = np.argsort(scores)[-n_elite:]
            elite = cand[elite_idx]
            mean = elite.mean(axis=0)
            std = elite.std(axis=0) + 1e-6
            if float(scores[elite_idx[-1]]) > best_s:
                best_s = float(scores[elite_idx[-1]])
                best_a = cand[elite_idx[-1]].copy()
        return best_a, best_s
