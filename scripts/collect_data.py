#!/usr/bin/env python3
"""数据采集脚本"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tqdm import tqdm
from src.utils.config import load_config
from src.simulation.env import SimulationEnv

def main():
    config = load_config("config/default.yaml")
    env = SimulationEnv(config)
    num_episodes = 100
    data = {"states": [], "actions": [], "next_states": []}

    for ep in tqdm(range(num_episodes), desc="Collecting data"):
        state = env.reset()
        action = np.random.randn(4).astype(np.float32)
        action[2] = np.random.uniform(0, 2 * np.pi)
        action[3] = np.random.uniform(10, 80)
        next_state, reward, done, info = env.step(action)
        data["states"].append(state)
        data["actions"].append(action)
        data["next_states"].append(next_state)

    save_path = "data/raw/push_data.npz"
    os.makedirs("data/raw", exist_ok=True)
    np.savez(save_path, states=np.array(data["states"]),
             actions=np.array(data["actions"]),
             next_states=np.array(data["next_states"]))
    print(f"Saved {len(data['states'])} samples to {save_path}")
    env.close()

if __name__ == "__main__":
    main()
