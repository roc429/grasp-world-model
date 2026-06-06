"""推动作执行器"""
from src.robot.base_arm import ArmController, Action
import numpy as np


class PushExecutor:
    """推动作执行器 - 将规划器输出的参数转为机械臂指令"""

    def __init__(self, arm: ArmController, config: dict):
        self.arm = arm
        self.config = config

    def execute(self, push_params: np.ndarray) -> bool:
        """
        执行推动作。

        Args:
            push_params: [start_x, start_y, direction_angle, distance]
        """
        start_x = float(push_params[0])
        start_y = float(push_params[1])
        angle = float(push_params[2])
        dist = float(push_params[3])
        z_push = self.config.get("desk_z", 0) + 10  # 略高于桌面

        action = Action(type="push", params={
            "start_x": start_x,
            "start_y": start_y,
            "direction_angle": angle,
            "distance": dist,
            "z_push": z_push,
        })
        return self.arm.execute_action(action)
