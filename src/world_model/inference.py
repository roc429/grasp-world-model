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

"""世界模型推理接口"""

import torch
import numpy as np
from typing import Tuple, Optional, Dict
from src.world_model.model import WorldModelMLP, WorldModelGRU


class WorldModel:
    """世界模型推理封装"""

    def __init__(
        self,
        model_path: str,
        config: dict,
        score_weights: Optional[Dict[str, float]] = None,
    ):
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

        self.model.load_state_dict(
            torch.load(model_path, map_location=self.device, weights_only=True)
        )
        self.model.to(self.device)
        self.model.eval()
        self.state_dim = model_cfg["state_dim"]
        self.action_dim = model_cfg["action_dim"]

        # 评分权重
        if score_weights is None:
            score_weights = {}
        self.graspable_weight = score_weights.get("graspable_weight", 1.0)
        self.distance_weight  = score_weights.get("distance_weight", 0.3)
        self.collision_penalty = score_weights.get("collision_penalty", -10.0)

    def predict(
        self,
        state: np.ndarray,
        action: np.ndarray,
        hidden: Optional[torch.Tensor] = None,
    ) -> Tuple[np.ndarray, float, Optional[torch.Tensor]]:
        """预测下一步状态并评分"""
        s = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        a = torch.FloatTensor(action).unsqueeze(0).to(self.device)
        with torch.no_grad():
            if isinstance(self.model, WorldModelGRU):
                pred, new_hidden = self.model(s, a, hidden)
            else:
                pred = self.model(s, a)
                new_hidden = None
        next_state = pred.squeeze(0).cpu().numpy()
        score = self._compute_score(next_state)
        return next_state, score, new_hidden

    def predict_trajectory(
        self,
        state: np.ndarray,
        action_sequence: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """沿动作序列 rollout 整条轨迹"""
        H = len(action_sequence)
        trajectory = np.zeros((H + 1, self.state_dim), dtype=np.float32)
        scores = np.zeros(H, dtype=np.float32)
        trajectory[0] = state

        current_state = state
        hidden = None
        for t in range(H):
            next_state, score, hidden = self.predict(
                current_state, action_sequence[t], hidden,
            )
            trajectory[t + 1] = next_state
            scores[t] = score
            current_state = next_state
        return trajectory, scores

    def _compute_score(self, state: np.ndarray) -> float:
        """
        评分函数 - 状态格式 (对齐 env):
          [0:3] target (x,y,z), [3:6] obstacle (x,y,z),
          [6:9] ee (x,y,z),    [9] gripper_state

        评分策略:
          + graspable_weight * (1 if gripper_open else 0)
          - distance_weight * dist(ee, target)  越小越好
          + collision_penalty * is_collision    障碍物贴近目标扣分
          - z_out_of_range * penalty            目标高度异常扣分
        """
        target_xyz  = state[0:3]
        obstacle_xyz = state[3:6]
        ee_xyz      = state[6:9]
        gripper_open = state[9] > 0.5 if len(state) > 9 else False

        score = 0.0

        # 1. 夹爪打开加分（准备好抓取）
        score += self.graspable_weight * (1.0 if gripper_open else 0.0)

        # 2. 末端到目标的距离惩罚
        dist_to_target = np.linalg.norm(ee_xyz[:2] - target_xyz[:2])
        score -= self.distance_weight * dist_to_target

        # 3. 碰撞惩罚: 障碍物与目标距离过近
        if np.any(obstacle_xyz[:2] != 0):
            obstacle_dist = np.linalg.norm(obstacle_xyz[:2] - target_xyz[:2])
            if obstacle_dist < 30.0 and obstacle_xyz[2] > 1.0:
                score += self.collision_penalty

        # 4. 目标高度合理性
        if target_xyz[2] < -50 or target_xyz[2] > 200:
            score -= 5.0

        return float(score)



