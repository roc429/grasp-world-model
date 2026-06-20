#!/usr/bin/env python3
"""数据预处理: 归一化、统计、质量检查

用法:
    python scripts/preprocess_data.py --input data/raw/push_data.npz --output data/processed/push_data_norm.npz
"""

import sys, os, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="data/raw/push_data.npz")
    parser.add_argument("--output", default="data/processed/push_data_norm.npz")
    parser.add_argument("--stats", default="data/processed/norm_stats.npz")
    args = parser.parse_args()

    d = np.load(args.input)
    states = d["states"].astype(np.float32)
    actions = d["actions"].astype(np.float32)
    next_states = d["next_states"].astype(np.float32)

    print(f"Loaded: {len(states)} samples")

    # 统计
    s_mean = states.mean(axis=0)
    s_std = states.std(axis=0) + 1e-8
    a_mean = actions.mean(axis=0)
    a_std = actions.std(axis=0) + 1e-8

    print(f"State  mean: {s_mean}")
    print(f"State  std:  {s_std}")
    print(f"Action mean: {a_mean}")
    print(f"Action std:  {a_std}")

    # 归一化
    states_norm = (states - s_mean) / s_std
    actions_norm = (actions - a_mean) / a_std
    next_states_norm = (next_states - s_mean) / s_std

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    np.savez(args.output,
             states=states_norm, actions=actions_norm,
             next_states=next_states_norm)
    np.savez(args.stats,
             s_mean=s_mean, s_std=s_std, a_mean=a_mean, a_std=a_std)
    print(f"Saved: {args.output}")
    print(f"Stats: {args.stats}")


if __name__ == "__main__":
    main()
