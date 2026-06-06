"""
W1 仿真环境验证脚本 -- 场景初始化 + 推动作 + 抓取测试

用法:
    python scripts/test_simulation.py [--layout 1|2|3]
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import argparse
import numpy as np
from src.utils.config import load_config
from src.simulation.env import SimulationEnv
from src.simulation.scene_builder import load_scene_config
from src.robot.sim_arm import SimArm
from src.robot.base_arm import Pose


def main():
    parser = argparse.ArgumentParser(description="仿真环境验证")
    parser.add_argument("--layout", type=int, default=2, choices=[1, 2, 3],
                        help="场景布局编号 (默认: 2)")
    args = parser.parse_args()

    # 1. 加载配置
    print("=" * 60)
    print("[1/8] 加载配置...")
    config = load_config("config/default.yaml")
    print(f"      场景布局: layout_{args.layout}")

    scene_config = load_scene_config(args.layout)
    print(f"      场景描述: {scene_config['description']}")

    # 2. 创建仿真环境
    print("\n[2/8] 创建 MuJoCo 仿真环境...")
    env = SimulationEnv(config)

    print("\n[3/8] 重置场景...")
    state = env.reset(layout_id=args.layout)
    print(f"      初始状态向量长度: {len(state)}")

    # 3. 创建仿真机械臂
    print("\n[4/8] 创建 SimArm...")
    arm = SimArm(env=env)
    arm.connect()

    # 4. 显示初始物体位置
    print("\n[5/8] 初始物体位置:")
    objects = env.get_objects_state()
    print(f"      目标物体:    x={objects['target'][0]:7.1f}  y={objects['target'][1]:7.1f}  z={objects['target'][2]:7.1f} mm")
    print(f"      障碍物:      x={objects['obstacle'][0]:7.1f}  y={objects['obstacle'][1]:7.1f}  z={objects['obstacle'][2]:7.1f} mm")
    print(f"      末端执行器:   x={objects['ee'][0]:7.1f}  y={objects['ee'][1]:7.1f}  z={objects['ee'][2]:7.1f} mm")

    # 5. 测试 PTP 运动
    print("\n[6/8] 测试 PTP 运动...")
    target_pose = Pose(200, 0, 100, 0)
    print(f"      目标: x={target_pose.x}, y={target_pose.y}, z={target_pose.z}")
    success = arm.move_to_pose(target_pose)
    current = arm.get_pose()
    print(f"      结果: {'OK' if success else 'FAIL'}, 当前位置: x={current.x:.1f}, y={current.y:.1f}, z={current.z:.1f}")

    # 6. 测试推动作
    print("\n[7/8] 测试推动作 (push)...")
    target_pos = env.get_objects_state()["target"]
    obstacle_pos = env.get_objects_state()["obstacle"]
    print(f"      推之前 - 目标: ({target_pos[0]:.1f}, {target_pos[1]:.1f})")
    print(f"      推之前 - 障碍: ({obstacle_pos[0]:.1f}, {obstacle_pos[1]:.1f})")

    # 从障碍物前方 20mm 处，以 45 度方向推 40mm
    push_start = np.array([obstacle_pos[0] - 20, obstacle_pos[1]])
    push_angle = np.pi / 4
    push_dist = 40.0

    print(f"      推起点: ({push_start[0]:.1f}, {push_start[1]:.1f}), "
          f"角度={np.degrees(push_angle):.0f}度, 距离={push_dist:.0f}mm")

    env.execute_push(
        start_xy_mm=push_start,
        direction_angle=push_angle,
        distance_mm=push_dist,
        z_push_mm=12.0,
    )

    target_after = env.get_objects_state()["target"]
    obstacle_after = env.get_objects_state()["obstacle"]
    print(f"      推之后 - 目标: ({target_after[0]:.1f}, {target_after[1]:.1f})")
    print(f"      推之后 - 障碍: ({obstacle_after[0]:.1f}, {obstacle_after[1]:.1f})")

    t_move = np.linalg.norm(target_after[:2] - target_pos[:2])
    o_move = np.linalg.norm(obstacle_after[:2] - obstacle_pos[:2])
    print(f"      目标移动: {t_move:.1f} mm, 障碍移动: {o_move:.1f} mm")
    if o_move > 1.0:
        print("      >> 推动作有效! 障碍物被推动了")
    else:
        print("      >> 障碍物几乎没动 (可能需要调参数)")

    # 7. 抓取-放置
    print("\n[8/8] 测试抓取-放置 (grasp & place)...")
    target = env.get_objects_state()["target"]
    print(f"      目标位置: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})")

    env.execute_grasp(obj_xy_mm=target[:2], obj_z_mm=target[2] + 10)
    print(f"      抓取完成, 夹爪={'闭合' if not env._gripper_open else '打开'}")

    placement = scene_config["placement_zone"]["position"]
    print(f"      放置到: ({placement[0]}, {placement[1]})")
    env.execute_place(
        target_xy_mm=np.array(placement[:2]),
        target_z_mm=float(placement[2]) if len(placement) > 2 else 20.0,
    )
    print(f"      放置完成, 夹爪={'闭合' if not env._gripper_open else '打开'}")

    # 最终状态
    final = env.get_objects_state()
    print(f"\n      最终 - 目标: x={final['target'][0]:.1f}, y={final['target'][1]:.1f}")
    print(f"      最终 - 障碍: x={final['obstacle'][0]:.1f}, y={final['obstacle'][1]:.1f}")

    placement_xy = np.array(placement[:2], dtype=np.float32)
    dist = np.linalg.norm(final["target"][:2] - placement_xy)
    print(f"      目标到放置区距离: {dist:.1f} mm")

    arm.disconnect()
    env.close()

    print("\n" + "=" * 60)
    print("W1 仿真环境验证完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
