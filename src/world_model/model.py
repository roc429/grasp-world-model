"""世界模型定义 (MLP/GRU 基础版)"""

import torch
import torch.nn as nn
from typing import Tuple


class WorldModelMLP(nn.Module):
    """基于 MLP 的世界模型: 预测 (s, a) -> delta_s"""

    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, state_dim),
        )

    def forward(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        x = torch.cat([state, action], dim=-1)
        delta = self.net(x)
        return state + delta


class WorldModelGRU(nn.Module):
    """基于 GRU 的世界模型: 带隐状态记忆"""

    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.state_dim = state_dim
        self.hidden_dim = hidden_dim
        self.input_proj = nn.Linear(state_dim + action_dim, hidden_dim)
        self.gru = nn.GRU(hidden_dim, hidden_dim, batch_first=True)
        self.output_proj = nn.Linear(hidden_dim, state_dim)

    def forward(self, state, action, hidden=None):
        x = torch.cat([state, action], dim=-1)
        x = self.input_proj(x).unsqueeze(1)
        out, hidden = self.gru(x, hidden)
        delta = self.output_proj(out.squeeze(1))
        return state + delta, hidden
