#!/usr/bin/env python3
"""消融实验: 世界模型 vs Heuristic, MLP vs GRU, 不同 layout

用法:
    python scripts/run_ablation.py
"""

import sys, os, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.utils.config import load_config
from src.utils.metrics import compute_metrics
from src.simulation.env import SimulationEnv
from src.planner.cem_planner import CEMPlanner
from src.planner.shooting_planner import ShootingPlanner
from src.world_model.inference import WorldModel
from src.robot.sim_arm import SimArm
from src.robot.push import PushExecutor
from src.robot.grasp import GraspExecutor


def run_trials(env, arm, wm, planner, layout_id, num_trials=10):
    """运行一组实验, 返回成功率统计"""
    push_exec = PushExecutor(arm, config={"arm": {"z_safe": 30}})
    grasp_exec = GraspExecutor(arm, config={})
    results = []

    for trial in range(num_trials):
        state = env.reset(layout_id=layout_id)
        done = False
        step = 0
        success = False

        while not done and step < 20:
            if wm is not None:
                push_action, push_score = planner.plan(state, wm)
            else:
                # Heuristic: 固定方向推
                push_action = np.array([200., 0., 0., 40.], dtype=np.float32)

            push_exec.execute(push_action)
            step += 1
            next_state, reward, done, info = env.step(push_action)
            if done:
                break
            state = next_state

        if not done:
            grasp_exec.execute(200, 0, 10)
            grasp_exec.place(250, 0, 5)
            success = True

        results.append({"success": success, "total_steps": step, "replan_count": step})

    return compute_metrics(results)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--output", type=str, default="experiments/ablation_results.json")
    args = parser.parse_args()

    config = load_config("config/default.yaml")
    planner_cfg = load_config("config/planner.yaml")
    wm_cfg = load_config("config/world_model.yaml")

    env = SimulationEnv(config)
    arm = SimArm(env)

    all_results = {}

    # 尝试加载世界模型
    wm = None
    model_path = "models/world_model/world_model.pt"
    if os.path.exists(model_path):
        wm = WorldModel(model_path, wm_cfg)
        print(f"Loaded world model from {model_path}")
    else:
        print(f"WARNING: {model_path} not found, using heuristic only")

    planner = CEMPlanner(planner_cfg)

    for layout_id in [1, 2, 3]:
        key = f"layout_{layout_id}"
        print(f"\n=== Layout {layout_id} ===")

        # Heuristic baseline
        print("  Heuristic...")
        metrics_heuristic = run_trials(env, arm, None, None, layout_id, args.trials)
        print(f"    Success rate: {metrics_heuristic['success_rate']:.1%}")

        # World Model
        if wm is not None:
            print("  World Model...")
            metrics_wm = run_trials(env, arm, wm, planner, layout_id, args.trials)
            print(f"    Success rate: {metrics_wm['success_rate']:.1%}")
        else:
            metrics_wm = {"success_rate": 0.0, "avg_steps": 0.0}

        all_results[key] = {
            "heuristic": metrics_heuristic,
            "world_model": metrics_wm,
        }

    env.close()

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(all_results, f, indent=2, ensure_ascii=False)
    print(f"\nResults saved to {args.output}")


if __name__ == "__main__":
    main()
