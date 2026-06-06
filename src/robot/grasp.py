"""抓取/放置动作执行器"""
from src.robot.base_arm import ArmController, Action


class GraspExecutor:
    """抓取执行器"""

    def __init__(self, arm: ArmController, config: dict):
        self.arm = arm
        self.config = config

    def execute(self, obj_x: float, obj_y: float, obj_z: float) -> bool:
        z_safe = self.config.get("z_safe", obj_z + 30)
        action = Action(type="grasp", params={
            "x": obj_x, "y": obj_y, "z": obj_z - 5, "r": 0,
            "z_safe": z_safe,
        })
        return self.arm.execute_action(action)

    def place(self, x: float, y: float, z: float) -> bool:
        z_safe = self.config.get("z_safe", z + 30)
        action = Action(type="place", params={
            "x": x, "y": y, "z": z, "r": 0,
            "z_safe": z_safe,
        })
        return self.arm.execute_action(action)
