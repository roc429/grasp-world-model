"""
W2 机械臂连接测试 -- 统一测试仿真/实机

用法:
    python scripts/test_arm_connection.py              # 仿真模式 (默认)
    python scripts/test_arm_connection.py --mode dobot --port COM3  # 实机模式
    python scripts/test_arm_connection.py --all-layouts  # 测试全部3种布局
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import argparse
import numpy as np
from src.utils.config import load_config
from src.robot.base_arm import Pose


def test_sim_arm(all_layouts=False):
    """测试仿真机械臂"""
    from src.simulation.env import SimulationEnv
    from src.simulation.scene_builder import load_scene_config
    from src.robot.sim_arm import SimArm

    config = load_config("config/default.yaml")
    env = SimulationEnv(config)

    layouts = [1, 2, 3] if all_layouts else [2]
    results = []

    for layout_id in layouts:
        scene = load_scene_config(layout_id)
        print(f"\n{'='*50}")
        print(f"Layout {layout_id}: {scene['description']}")
        print('='*50)

        state = env.reset(layout_id=layout_id)
        arm = SimArm(env=env)
        arm.connect()

        # 1. 连接 + 位姿
        print("[1/3] 连接测试...")
        pose = arm.get_pose()
        print(f"      末端位置: ({pose.x:.1f}, {pose.y:.1f}, {pose.z:.1f}) mm")

        # 2. PTP 运动
        print("[2/3] PTP 运动测试...")
        arm.move_to_pose(Pose(200, 0, 100, 0))
        print("      OK")

        # 3. 读取物体
        print("[3/3] 场景物体位置:")
        objs = env.get_objects_state()
        print(f"      目标: ({objs['target'][0]:.1f}, {objs['target'][1]:.1f})")
        print(f"      障碍: ({objs['obstacle'][0]:.1f}, {objs['obstacle'][1]:.1f})")
        print(f"      放置区: {scene['placement_zone']['position'][:2]}")

        arm.disconnect()
        results.append(True)

    env.close()

    print(f"\n{'='*50}")
    print(f"结果: {sum(results)}/{len(results)} 布局通过测试")
    print('='*50)
    return all(results)


def test_dobot_arm(port=""):
    """测试 Dobot 实机（需要 DobotDllType.py 和硬件连接）"""
    try:
        from src.robot.dobot_arm import DobotArm, DOBOT_DLL_AVAILABLE
    except ImportError as e:
        print(f"[ERROR] 导入失败: {e}")
        return False

    if not DOBOT_DLL_AVAILABLE:
        print("\n[ERROR] DobotDllType.py 未找到！")
        print("请从魔术师资料包复制 DobotDllType.py 到 src/robot/ 目录")
        print("或使用仿真模式: python scripts/test_arm_connection.py --mode sim")
        return False

    arm = DobotArm()
    try:
        print(f"连接 Dobot (port={port or 'auto'})...")
        if not arm.connect(port=port):
            return False

        pose = arm.get_pose()
        print(f"当前位姿: x={pose.x:.1f} y={pose.y:.1f} z={pose.z:.1f} r={pose.r:.1f}")

        joints = arm.get_joint_angles()
        print(f"关节角度: J1={joints['j1']:.1f} J2={joints['j2']:.1f} "
              f"J3={joints['j3']:.1f} J4={joints['j4']:.1f}")

        print("小范围运动测试...")
        arm.move_to_pose(Pose(pose.x, pose.y, pose.z + 15, pose.r))
        arm.move_to_pose(pose)

        arm.disconnect()
        print("Dobot 测试通过!")
        return True
    except Exception as e:
        print(f"[ERROR] {e}")
        arm.emergency_stop()
        arm.disconnect()
        return False


def main():
    parser = argparse.ArgumentParser(description="机械臂连接测试")
    parser.add_argument("--mode", choices=["sim", "dobot"], default="sim",
                        help="测试模式: sim=仿真, dobot=实机 (默认: sim)")
    parser.add_argument("--port", default="",
                        help="Dobot 串口号 (如 COM3, /dev/ttyUSB0)")
    parser.add_argument("--all-layouts", action="store_true",
                        help="仿真模式: 测试全部 3 种布局")
    args = parser.parse_args()

    if args.mode == "dobot":
        success = test_dobot_arm(args.port)
    else:
        success = test_sim_arm(all_layouts=args.all_layouts)

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
