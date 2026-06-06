#!/usr/bin/env python3
"""完整任务执行脚本 - 推-抓-放置 全流程"""
import sys, os, argparse, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.utils.config import load_config
from src.utils.logger import setup_logger, ExperimentLogger
from src.utils.metrics import compute_metrics
from src.simulation.env import SimulationEnv
from src.planner.cem_planner import CEMPlanner
from src.planner.shooting_planner import ShootingPlanner
from src.world_model.inference import WorldModel
from src.robot.sim_arm import SimArm
from src.robot.push import PushExecutor
from src.robot.grasp import GraspExecutor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--layout", type=int, default=1)
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--planner", type=str, default="cem")
    args = parser.parse_args()

    config = load_config("config/default.yaml")
    logger = setup_logger("run_task")
    logger.info(f"Starting: layout={args.layout}, trials={args.trials}")

    env = SimulationEnv(config)
    arm = SimArm(env)
    wm = WorldModel("models/world_model/world_model.pt",
                    load_config("config/world_model.yaml"))

    planner = (CEMPlanner(load_config("config/planner.yaml"))
               if args.planner == "cem"
               else ShootingPlanner(load_config("config/planner.yaml")))

    push_exec = PushExecutor(arm, config)
    grasp_exec = GraspExecutor(arm, config)
    exp_logger = ExperimentLogger(f"layout{args.layout}_{args.planner}", config)
    results = []

    for trial in range(args.trials):
        logger.info(f"=== Trial {trial + 1}/{args.trials} ===")
        state = env.reset(layout_id=args.layout)
        done = False
        step = 0
        success = False
        replan_count = 0

        while not done and step < 20:
            push_action, push_score = planner.plan(state, wm)
            logger.info(f"Push score: {push_score:.3f}")
            push_exec.execute(push_action)
            step += 1
            next_state, reward, done, info = env.step(push_action)
            replan_count += 1
            if done:
                break
            state = next_state

        if not done:
            grasp_exec.execute(200, 0, 10)
            grasp_exec.place(250, 0, 5)
            success = True

        exp_logger.log_step(step, {
            "trial": trial, "success": success, "replan_count": replan_count,
        })
        results.append({
            "success": success, "total_steps": step, "replan_count": replan_count,
        })

    metrics = compute_metrics(results)
    exp_logger.log_result(True, metrics)
    exp_logger.save()
    logger.info(f"Success rate: {metrics['success_rate']:.1%}")
    env.close()


if __name__ == "__main__":
    main()
