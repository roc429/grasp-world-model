#!/usr/bin/env python3
"""规划器单元测试"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.utils.config import load_config
from src.planner.shooting_planner import ShootingPlanner
from src.planner.cem_planner import CEMPlanner
from src.world_model.inference import WorldModel

def test_planner(planner_type: str = "shooting"):
    config = load_config("config/planner.yaml")
    wm_config = load_config("config/world_model.yaml")
    wm = WorldModel("models/world_model/world_model.pt", wm_config)
    if planner_type == "cem":
        planner = CEMPlanner(config)
    else:
        planner = ShootingPlanner(config)
    test_state = np.array([150, 0, 15, 200, 0, 0, 0, 0, 0, 0], dtype=np.float32)
    best_action, best_score = planner.plan(test_state, wm)
    print(f"Planner: {planner_type}, Score: {best_score:.3f}")
    return best_action is not None

if __name__ == "__main__":
    test_planner("shooting")
