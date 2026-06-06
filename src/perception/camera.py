"""相机接口 - 支持仿真真值和实机 RGB-D 相机"""

from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np


class CameraInterface(ABC):
    """相机抽象基类"""

    @abstractmethod
    def get_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取 RGB + Depth 图像"""
        pass

    @abstractmethod
    def get_intrinsics(self) -> dict:
        """获取相机内参"""
        pass


class SimCamera(CameraInterface):
    """仿真相机 - 从 MuJoCo/PyBullet 获取真值"""

    def __init__(self, env):
        self._env = env

    def get_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        # TODO: 从仿真环境获取 RGB-D
        raise NotImplementedError

    def get_intrinsics(self) -> dict:
        # TODO: 返回仿真相机内参
        raise NotImplementedError
