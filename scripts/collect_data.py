#!/usr/bin/env python3
"""Data collection script."""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.simulation.env import PushGraspEnv
from src.utils.config import load_config
from tqdm import tqdm

def main():
    config = load_config("config/default.yaml")
    env = PushGraspEnv(config)
    total = 3000
    data = {"states": [], "actions": [], "next_states": []}
    per_layout = total // 3
    for lid in [1, 2, 3]:
        for _ in tqdm(range(per_layout), desc="Layout {}".format(lid)):
            state = env.reset(layout_id=lid)
            obj = env.get_objects_state()["target"]
            action = np.array([
                obj[0] + np.random.uniform(-0.02, 0.02),
                obj[1] + np.random.uniform(-0.02, 0.02),
                np.random.uniform(0, 2*np.pi),
                np.random.uniform(0.01, 0.08)
            ], dtype=np.float32)
            ns, _, _, _ = env.step(action)
            data["states"].append(state)
            data["actions"].append(action)
            data["next_states"].append(ns)
    os.makedirs("data/raw", exist_ok=True)
    np.savez("data/raw/push_data.npz",
             states=np.array(data["states"]),
             actions=np.array(data["actions"]),
             next_states=np.array(data["next_states"]))
    print("Saved {} samples".format(len(data["states"])))
    env.close()

if __name__ == "__main__":
    main()
