"""规划器抽象基类"""
from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple


class Planner(ABC):
    """规划器抽象基类"""

    def __init__(self, config: dict):
        self.config = config
        self.horizon = config.get("horizon", 5)

    @abstractmethod
    def plan(self, state: np.ndarray,
             world_model: "WorldModel") -> Tuple[np.ndarray, float]:
        """
        Args:
            state: 当前状态向量 (state_dim,)
            world_model: 世界模型实例

        Returns:
            best_action: 最优第一步动作 (action_dim,)
            best_score: 该动作的评分
        """
        pass
