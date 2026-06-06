"""世界模型训练数据集"""

import torch
from torch.utils.data import Dataset
import numpy as np
from typing import Tuple


class PushGraspDataset(Dataset):
    """推-抓交互数据集, 每条: (state, action, next_state)"""

    def __init__(self, data_path: str = None,
                 states: np.ndarray = None,
                 actions: np.ndarray = None,
                 next_states: np.ndarray = None):
        if data_path:
            d = np.load(data_path)
            self.states = torch.FloatTensor(d["states"])
            self.actions = torch.FloatTensor(d["actions"])
            self.next_states = torch.FloatTensor(d["next_states"])
        else:
            self.states = torch.FloatTensor(states)
            self.actions = torch.FloatTensor(actions)
            self.next_states = torch.FloatTensor(next_states)

    def __len__(self) -> int:
        return len(self.states)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, ...]:
        return self.states[idx], self.actions[idx], self.next_states[idx]
