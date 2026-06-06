"""
W2 批量评估脚本 -- 3 种布局全流程测试 (推+抓+放)

用法:
    python scripts/evaluate.py              # 测试全部 3 种布局
    python scripts/evaluate.py --layout 2   # 只测试布局 2
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


def evaluate_layout(env, layout_id, verbose=True):
    """
    对单个布局执行完整的推-抓-放流程。

    返回: dict with metrics
    """
    scene = load_scene_config(layout_id)
    state = env.reset(layout_id=layout_id)
    arm = SimArm(env=env)
    arm.connect()

    if verbose:
        print(f"\n{'='*55}")
        print(f"Layout {layout_id}: {scene['description']}")
        print(f"{'='*55}")

    objs = env.get_objects_state()
    target_init = objs["target"].copy()
    obstacle_init = objs["obstacle"].copy()
    placement_xy = np.array(
        scene["placement_zone"]["position"][:2], dtype=np.float32
    )

    if verbose:
        print(f"  [初始] 目标: ({target_init[0]:.0f},{target_init[1]:.0f})  "
              f"障碍: ({obstacle_init[0]:.0f},{obstacle_init[1]:.0f})")

    # -- 阶段 1: 判断是否需要推 --
    target = env.get_objects_state()["target"]
    obstacle = env.get_objects_state()["obstacle"]
    dist_to_obstacle = np.linalg.norm(target[:2] - obstacle[:2])
    need_push = (dist_to_obstacle < 80.0)
    push_effect = 0.0

    # 末端执行器圆柱半径 12mm，需要确保接触到障碍物
    CYLINDER_RADIUS = 12.0

    if need_push and layout_id in [2, 3]:
        if verbose:
            print(f"  [策略] 目标被遮挡 (距障碍{dist_to_obstacle:.0f}mm)，先推")

        obstacle_pos = env.get_objects_state()["obstacle"]

        if layout_id == 2:
            # Layout 2: 障碍物在 (180,0)，目标在 (230,0)
            # 从障碍物前方推，向 Y+ 方向扫开
            push_start = np.array([
                obstacle_pos[0] - 5,      # 紧贴障碍物前方
                obstacle_pos[1] - CYLINDER_RADIUS,
            ])
            push_angle = np.pi / 2        # 90度，向 Y+ 推开
            push_dist = 60.0
        else:
            # Layout 3: 目标在 (80,-110)，障碍在 (150,-80)
            # 从侧面推，向 Y- 方向扫开
            push_start = np.array([
                obstacle_pos[0] - 5,
                obstacle_pos[1] + CYLINDER_RADIUS + 2,
            ])
            push_angle = -np.pi / 2       # -90度，向 Y- 推开
            push_dist = 50.0

        if verbose:
            print(f"         推起点({push_start[0]:.0f},{push_start[1]:.0f}) "
                  f"角度{np.degrees(push_angle):.0f}deg 距离{push_dist:.0f}mm")

        env.execute_push(
            start_xy_mm=push_start,
            direction_angle=push_angle,
            distance_mm=push_dist,
            z_push_mm=15.0,
        )

        obstacle_after = env.get_objects_state()["obstacle"]
        push_effect = np.linalg.norm(obstacle_after[:2] - obstacle_pos[:2])

        if verbose:
            print(f"         障碍移动: {push_effect:.1f}mm  "
                  f"({obstacle_pos[0]:.0f},{obstacle_pos[1]:.0f}) -> "
                  f"({obstacle_after[0]:.0f},{obstacle_after[1]:.0f})")

    elif layout_id == 1:
        if verbose:
            print("  [策略] 无障碍阻挡，直接抓取")

    # -- 阶段 2: 抓取 --
    target = env.get_objects_state()["target"]
    if verbose:
        print(f"  [抓取] 目标: ({target[0]:.0f},{target[1]:.0f},{target[2]:.0f})")

    env.execute_grasp(obj_xy_mm=target[:2], obj_z_mm=target[2] + 5)
    gripped = not env._gripper_open

    # -- 阶段 3: 放置 --
    if verbose:
        print(f"  [放置] 目标区: ({placement_xy[0]:.0f},{placement_xy[1]:.0f})")
    env.execute_place(target_xy_mm=placement_xy, target_z_mm=20.0)

    # -- 阶段 4: 评估 --
    final_objs = env.get_objects_state()
    target_final = final_objs["target"]
    dist_to_goal = np.linalg.norm(target_final[:2] - placement_xy)
    target_moved = np.linalg.norm(target_final[:2] - target_init[:2])

    if verbose:
        print(f"  [结果] 目标: ({target_final[0]:.0f},{target_final[1]:.0f})  "
              f"距目标区: {dist_to_goal:.0f}mm")
        status = "OK" if dist_to_goal < 80 else "PARTIAL"
        print(f"  [评估] {status}  "
              f"(推={push_effect:.1f}mm 抓={'OK' if gripped else 'FAIL'})")

    arm.disconnect()

    return {
        "layout": layout_id,
        "success": dist_to_goal < 80,
        "push_effect_mm": float(push_effect),
        "grasp_ok": gripped,
        "dist_to_goal_mm": float(dist_to_goal),
        "target_moved_mm": float(target_moved),
    }


def main():
    parser = argparse.ArgumentParser(description="批量评估推-抓-放流程")
    parser.add_argument("--layout", type=int, default=0, choices=[0, 1, 2, 3],
                        help="测试指定布局 (0=全部)")
    parser.add_argument("--trials", type=int, default=1,
                        help="每种布局重复次数")
    args = parser.parse_args()

    config = load_config("config/default.yaml")
    env = SimulationEnv(config)

    layouts = [args.layout] if args.layout != 0 else [1, 2, 3]

    all_results = []
    for layout_id in layouts:
        for trial in range(args.trials):
            result = evaluate_layout(env, layout_id, verbose=True)
            all_results.append(result)

    env.close()

    # -- 汇总 --
    print(f"\n{'='*55}")
    print("评估汇总")
    print(f"{'='*55}")
    print(f"{'Layout':<8} {'推效果':<10} {'抓取':<8} {'距目标':<10} {'结果'}")
    print(f"{'-'*45}")
    for r in all_results:
        status = "OK" if r["success"] else "PARTIAL"
        print(f"{r['layout']:<8} {r['push_effect_mm']:<10.1f} "
              f"{'OK' if r['grasp_ok'] else 'FAIL':<8} "
              f"{r['dist_to_goal_mm']:<10.1f} {status}")

    success_count = sum(1 for r in all_results if r["success"])
    print(f"\n成功率: {success_count}/{len(all_results)}")
    print(f"{'='*55}")


if __name__ == "__main__":
    main()
