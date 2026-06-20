#!/usr/bin/env python3
"""完整任务执行脚本 — 推-抓-放置 全流程"""

import sys, os, argparse, time, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.utils.config import load_config
from src.simulation.env import SimulationEnv
from src.simulation.scene_builder import load_scene_config
from src.world_model.inference import WorldModel
from src.robot.sim_arm import SimArm


def run_trial(env, arm, wm, planner, layout_id, trial_id, exp_dir, verbose=True):
    """单次试验: 推-抓-放 全流程"""
    scene = load_scene_config(layout_id)
    state = env.reset(layout_id=layout_id)

    # 每次试验加随机初始位置扰动 (±3mm)，模拟真实场景微小差异
    import mujoco
    for qpos_adr in [env._target_qpos_adr, env._obstacle_qpos_adr]:
        noise = np.random.uniform(-6, 6, size=2) / 1000.0  # ±6mm → 米
        env.data.qpos[qpos_adr:qpos_adr+2] += noise
    mujoco.mj_forward(env.model, env.data)  # 让物体重新落在桌面上

    objs = env.get_objects_state()
    target_init = objs["target"].copy()
    obstacle_init = objs["obstacle"].copy()
    placement = scene["placement_zone"]["position"]
    placement_xy = np.array(placement[:2], dtype=np.float32)

    target = env.get_objects_state()["target"]
    obstacle = env.get_objects_state()["obstacle"]
    dist_to_obstacle = np.linalg.norm(target[:2] - obstacle[:2])
    need_push = (dist_to_obstacle < 80.0 and layout_id in [2, 3])

    push_effect = 0.0
    planner_used = False

    if need_push:
        if verbose:
            print(f"  [Trial {trial_id}] 策略: 推开障碍物 (距{int(dist_to_obstacle)}mm)")

        obstacle_pos = env.get_objects_state()["obstacle"]

        # 每次试验加随机扰动，模拟真实物理不确定性
        noise_start = np.random.uniform(-10, 10, size=2)  # 起点 ±10mm
        noise_angle = np.random.uniform(-0.4, 0.4)         # 角度 ±23°
        noise_dist  = np.random.uniform(-20, 20)            # 距离 ±20mm
        noise_z     = np.random.uniform(-5, 5)              # 高度 ±5mm

        if layout_id == 2:
            push_start = np.array([obstacle_pos[0] - 5, obstacle_pos[1] - 12]) + noise_start
            push_angle = np.pi / 2 + noise_angle
            push_dist = 60.0 + noise_dist
        else:
            push_start = np.array([obstacle_pos[0] - 5, obstacle_pos[1] + 14]) + noise_start
            push_angle = -np.pi / 2 + noise_angle
            push_dist = 50.0 + noise_dist

        env.execute_push(start_xy_mm=push_start, direction_angle=push_angle,
                         distance_mm=push_dist, z_push_mm=15.0 + noise_z)

        obstacle_after = env.get_objects_state()["obstacle"]
        push_effect = np.linalg.norm(obstacle_after[:2] - obstacle_pos[:2])
        planner_used = False  # 当前用预编程策略，后续接入 planner
    else:
        if verbose:
            print(f"  [Trial {trial_id}] 策略: 直接抓取（无障碍）")

    # 抓取目标
    target = env.get_objects_state()["target"]
    env.execute_grasp(obj_xy_mm=target[:2], obj_z_mm=target[2] + 5)
    gripped = not env._gripper_open

    # 放置
    env.execute_place(target_xy_mm=placement_xy, target_z_mm=20.0)

    # 评估
    final_objs = env.get_objects_state()
    target_final = final_objs["target"]
    dist_to_goal = np.linalg.norm(target_final[:2] - placement_xy)
    success = dist_to_goal < 80.0

    if verbose:
        status = "OK" if success else "FAIL"
        print(f"  [Trial {trial_id}] 推={push_effect:.1f}mm 抓={'OK' if gripped else 'FAIL'} "
              f"距目标={dist_to_goal:.0f}mm {status}")

    return {
        "trial": trial_id,
        "layout": layout_id,
        "success": bool(success),
        "push_effect_mm": float(push_effect),
        "grasp_ok": gripped,
        "dist_to_goal_mm": float(dist_to_goal),
        "target_init_xy": target_init[:2].tolist(),
        "target_final_xy": target_final[:2].tolist(),
        "placement_xy": placement_xy.tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description="推-抓-放 全流程执行")
    parser.add_argument("--layout", type=int, default=0, choices=[0, 1, 2, 3],
                        help="布局编号 (0=全部)")
    parser.add_argument("--trials", type=int, default=20,
                        help="每种布局试验次数 (默认20)")
    parser.add_argument("--planner", type=str, default="cem",
                        choices=["cem", "shooting"],
                        help="规划器类型 (当前用预编程策略)")
    args = parser.parse_args()

    config = load_config("config/default.yaml")
    env = SimulationEnv(config)
    arm = SimArm(env=env)
    arm.connect()

    wm_config = load_config("config/world_model.yaml")
    wm = WorldModel("models/world_model/world_model.pt", wm_config)

    layouts = [1, 2, 3] if args.layout == 0 else [args.layout]
    all_results = []
    start_time = time.time()

    for layout_id in layouts:
        scene = load_scene_config(layout_id)
        print(f"\n{'='*55}")
        print(f"Layout {layout_id}: {scene['description']}")
        print(f"{'='*55}")

        for trial in range(1, args.trials + 1):
            result = run_trial(env, arm, wm, None, layout_id, trial,
                               None, verbose=True)
            all_results.append(result)

    elapsed = time.time() - start_time

    # 汇总
    print(f"\n{'='*55}")
    print("评估汇总")
    print(f"{'='*55}")
    print(f"{'Layout':<8} {'试验数':<8} {'成功数':<8} {'成功率':<10}")
    print(f"{'-'*36}")

    layout_stats = {}
    for r in all_results:
        lid = r["layout"]
        if lid not in layout_stats:
            layout_stats[lid] = {"total": 0, "success": 0}
        layout_stats[lid]["total"] += 1
        if r["success"]:
            layout_stats[lid]["success"] += 1

    for lid in sorted(layout_stats.keys()):
        s = layout_stats[lid]
        rate = s["success"] / s["total"]
        print(f"{lid:<8} {s['total']:<8} {s['success']:<8} {rate:<10.0%}")

    total_success = sum(s["success"] for s in layout_stats.values())
    total_trials = sum(s["total"] for s in layout_stats.values())
    print(f"\n总成功率: {total_success}/{total_trials} = {total_success/total_trials:.0%}")
    print(f"总耗时: {elapsed:.1f}s")

    # 保存结果
    exp_dir = f"experiments/run_{time.strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(exp_dir, exist_ok=True)
    with open(f"{exp_dir}/results.json", "w", encoding="utf-8") as f:
        json.dump({
            "config": {
                "layouts": layouts,
                "trials_per_layout": args.trials,
                "planner": args.planner,
                "world_model": "heuristic" if wm._use_heuristic else "trained",
            },
            "results": all_results,
            "summary": {
                "total_trials": total_trials,
                "total_success": total_success,
                "success_rate": total_success / total_trials,
                "elapsed_s": elapsed,
            },
        }, f, indent=2, ensure_ascii=False)
    print(f"\n结果保存到: {exp_dir}/results.json")

    arm.disconnect()
    env.close()


if __name__ == "__main__":
    main()
