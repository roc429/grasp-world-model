"""世界模型推理接口"""

import torch
import numpy as np
from typing import Tuple
from src.world_model.model import WorldModelMLP, WorldModelGRU


class WorldModel:
    """世界模型推理封装"""

    def __init__(self, model_path: str, config: dict):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model_cfg = config["model"]

        if model_cfg["type"] == "mlp_gru":
            self.model = WorldModelGRU(
                state_dim=model_cfg["state_dim"],
                action_dim=model_cfg["action_dim"],
                hidden_dim=model_cfg["hidden_dim"],
            )
        else:
            self.model = WorldModelMLP(
                state_dim=model_cfg["state_dim"],
                action_dim=model_cfg["action_dim"],
                hidden_dim=model_cfg["hidden_dim"],
            )

        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()
        self.state_dim = model_cfg["state_dim"]
        self.action_dim = model_cfg["action_dim"]

    def predict(self, state: np.ndarray, action: np.ndarray) -> Tuple[np.ndarray, float]:
        s = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        a = torch.FloatTensor(action).unsqueeze(0).to(self.device)
        with torch.no_grad():
            pred, _ = self.model(s, a)
        next_state = pred.squeeze(0).cpu().numpy()
        score = self._compute_score(next_state)
        return next_state, score

    def predict_trajectory(self, state: np.ndarray,
                           action_sequence: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        H = len(action_sequence)
        trajectory = np.zeros((H + 1, self.state_dim), dtype=np.float32)
        scores = np.zeros(H, dtype=np.float32)
        trajectory[0] = state
        current_state = state
        for t in range(H):
            next_state, score = self.predict(current_state, action_sequence[t])
            trajectory[t + 1] = next_state
            scores[t] = score
            current_state = next_state
        return trajectory, scores

    def _compute_score(self, state: np.ndarray) -> float:
        return 0.5  # TODO: 实现更合理的评分函数
