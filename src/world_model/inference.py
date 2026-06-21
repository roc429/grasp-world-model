"""世界模型推理接口"""

import os
import numpy as np
from typing import Tuple
# torch 延迟导入 — 启发式模型不需要 torch


class HeuristicWorldModel:
    """
    启发式世界模型 — 基于简单物理假设预测推动作效果。

    不需要训练，用于:
      - 赵中赐训练模型之前的基线测试
      - 规划器开发调试
      - 消融实验的 baseline

    预测逻辑:
      - 假设物体沿推的方向线性移动，移动距离≈推的距离×0.6（经验摩擦系数）
      - 障碍物和目标物体会互相影响（简化处理）
    """

    def __init__(self, state_dim=10, action_dim=4, friction_coef=0.6):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.friction_coef = friction_coef

    def predict(self, state: np.ndarray, action: np.ndarray) -> Tuple[np.ndarray, float]:
        next_state = state.copy()
        push_x, push_y = action[0], action[1]
        push_dir = action[2]
        push_dist = action[3] * self.friction_coef

        # 判断推的是目标还是障碍物：找离推起点更近的那个
        target_xy = state[0:2]
        obstacle_xy = state[3:5]
        push_start = np.array([push_x, push_y])
        dist_to_target = np.linalg.norm(target_xy - push_start)
        dist_to_obstacle = np.linalg.norm(obstacle_xy - push_start)

        if dist_to_obstacle < dist_to_target and dist_to_obstacle < 50:
            # 推的是障碍物
            next_state[3] += push_dist * np.cos(push_dir)
            next_state[4] += push_dist * np.sin(push_dir)
        else:
            # 推的是目标物体
            next_state[0] += push_dist * np.cos(push_dir)
            next_state[1] += push_dist * np.sin(push_dir)

        score = self._compute_score(next_state)
        return next_state, score

    def predict_trajectory(self, state: np.ndarray,
                           action_sequence: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        H = len(action_sequence)
        trajectory = np.zeros((H + 1, self.state_dim), dtype=np.float32)
        scores = np.zeros(H, dtype=np.float32)
        trajectory[0] = state
        current_state = state.copy()
        for t in range(H):
            next_state, score = self.predict(current_state, action_sequence[t])
            trajectory[t + 1] = next_state
            scores[t] = score
            current_state = next_state
        return trajectory, scores

    def _compute_score(self, state: np.ndarray) -> float:
        # 目标越靠近场景中心（假设放置区在远处）分数越高
        target_x = state[0]
        return float(np.clip(target_x / 300.0, 0.0, 1.0))


class WorldModel:
    """
    世界模型推理封装。

    加载策略:
      - 如果 model_path 文件存在 → 加载训练好的 MLP/GRU 模型
      - 如果 model_path 不存在 → 自动回退到 HeuristicWorldModel（不依赖训练）
    """

    def __init__(self, model_path: str, config: dict):
        self.config = config

        if not os.path.exists(model_path):
            print(f"[WorldModel] 模型文件不存在 ({model_path})，使用 HeuristicWM 回退")
            model_cfg = config.get("world_model", config.get("model", {}))
            self._heuristic = HeuristicWorldModel(
                state_dim=model_cfg.get("state_dim", 10),
                action_dim=model_cfg.get("action_dim", 4),
            )
            self._use_heuristic = True
            self.state_dim = self._heuristic.state_dim
            self.action_dim = self._heuristic.action_dim
            return

        import torch
        from src.world_model.model import WorldModelMLP, WorldModelGRU
        model_cfg = config.get("world_model", config.get("model", {}))

        if model_cfg.get("type") == "mlp_gru":
            self.model = WorldModelGRU(
                state_dim=model_cfg["state_dim"],
                action_dim=model_cfg["action_dim"],
                hidden_dim=model_cfg.get("hidden_dim", 256),
            )
        else:
            self.model = WorldModelMLP(
                state_dim=model_cfg["state_dim"],
                action_dim=model_cfg["action_dim"],
                hidden_dim=model_cfg.get("hidden_dim", 256),
            )

        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()
        self.state_dim = model_cfg["state_dim"]
        self.action_dim = model_cfg["action_dim"]

    def predict(self, state: np.ndarray, action: np.ndarray) -> Tuple[np.ndarray, float]:
        if self._use_heuristic:
            return self._heuristic.predict(state, action)

        s = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        a = torch.FloatTensor(action).unsqueeze(0).to(self.device)
        with torch.no_grad():
            pred, _ = self.model(s, a)
        next_state = pred.squeeze(0).cpu().numpy()
        score = self._compute_score(next_state)
        return next_state, score

    def predict_trajectory(self, state: np.ndarray,
                           action_sequence: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self._use_heuristic:
            return self._heuristic.predict_trajectory(state, action_sequence)

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
        return float(np.clip(state[0] / 300.0, 0.0, 1.0))
