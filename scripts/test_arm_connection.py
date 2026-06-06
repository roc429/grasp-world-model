#!/usr/bin/env python3
"""机械臂连接测试脚本"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.utils.config import load_config
from src.robot.base_arm import Pose


def main():
    config = load_config("config/default.yaml")
    arm_type = config["arm"]["type"]

    if arm_type == "dobot":
        from src.robot.dobot_arm import DobotArm
        arm = DobotArm()
    else:
        from src.robot.sim_arm import SimArm
        arm = SimArm()

    print(f"Arm type: {arm_type}")
    if arm.connect():
        print("Connected!")
        pose = arm.get_pose()
        print(f"Current pose: x={pose.x:.1f}, y={pose.y:.1f}, z={pose.z:.1f}")
        arm.home()
        arm.move_to_pose(Pose(200, 0, 50, 0))
        arm.home()
        arm.disconnect()
        print("Test passed!")
    else:
        print("Connection failed!")


if __name__ == "__main__":
    main()
