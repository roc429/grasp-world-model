#!/usr/bin/env python3
"""数据采集脚本 - 在 MuJoCo 仿真中多步轨迹采集

用法:
    python scripts/collect_data.py --episodes 400 --steps 15

输出:
    data/raw/push_data.npz  (states, actions, next_states)
"""

import sys
import os
import argparse
import numpy as np
from tqdm import tqdm

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.utils.config import load_config
from src.simulation.env import SimulationEnv


def sample_push_action(
    action_dim: int = 4,
    target_xy=None,
    obstacle_xy=None,
    layout_id: int = 1,
) -> np.ndarray:
    """
    采样随机推动作 - 推起点对齐物体实际位置。

    动作向量 (4,):
      [0]: push_start_x  - 推起点 x (mm)
      [1]: push_start_y  - 推起点 y (mm)
      [2]: push_angle    - 推方向角度 (radians, 0~2pi)
      [3]: push_distance - 推距离 (mm, 30~80)

    策略:
      - Layout 1 (无障碍): 推 target 自身
      - Layout 2/3 (有障碍): 推 obstacle 清障
    """
    # 选择推哪个物体
    if layout_id == 1:
        obj_xy = target_xy
    else:
        obj_xy = obstacle_xy if obstacle_xy is not None else target_xy

    # 先定推方向(随机), 再定起点 = 物体后方 20mm, 保证推线穿过物体
    angle = np.random.uniform(0, 2 * np.pi)
    distance = np.random.uniform(30, 80)

    if obj_xy is not None:
        # 起点在推方向的反方向偏移 20mm, 则推线必穿过物体中心
        start_x = obj_xy[0] - 20.0 * np.cos(angle)
        start_y = obj_xy[1] - 20.0 * np.sin(angle)
    else:
        start_x = 200.0
        start_y = 0.0

    action = np.zeros(action_dim, dtype=np.float32)
    action[0] = start_x
    action[1] = start_y
    action[2] = angle
    action[3] = distance
    return action

def main():
    parser = argparse.ArgumentParser(description="采集推-抓任务训练数据")
    parser.add_argument("--episodes", type=int, default=400,
                        help="采集 episode 数量 (default: 400)")
    parser.add_argument("--steps", type=int, default=15,
                        help="每个 episode 的最大步数 (default: 15)")
    parser.add_argument("--layouts", type=int, nargs="+", default=[1, 2, 3],
                        help="场景布局 (default: 1 2 3)")
    parser.add_argument("--output", type=str,
                        default="data/raw/push_data.npz",
                        help="输出路径")
    parser.add_argument("--seed", type=int, default=42,
                        help="随机种子")
    args = parser.parse_args()

    np.random.seed(args.seed)

    print("=" * 60)
    print("数据采集配置:")
    print(f"  Episodes: {args.episodes}")
    print(f"  Max steps/episode: {args.steps}")
    print(f"  Layouts: {args.layouts}")
    print(f"  Output: {args.output}")
    print("=" * 60)

    config = load_config("config/default.yaml")
    env = SimulationEnv(config)

    states_list = []
    actions_list = []
    next_states_list = []
    total_steps = 0

    pbar = tqdm(total=args.episodes, desc="Collecting episodes")

    for ep in range(args.episodes):
        layout_id = int(np.random.choice(args.layouts))
        state = env.reset(layout_id=layout_id)

        episode_len = np.random.randint(5, args.steps + 1)

        for step in range(episode_len):
            # 每步刷新物体位置(物体被推动后会移位)
            obj_state = env.get_objects_state()
            target_xy = obj_state["target"][:2] if "target" in obj_state else None
            obs = obj_state.get("obstacle", None)
            obstacle_xy = obs[:2] if obs is not None else None

            action = sample_push_action(
                action_dim=config.get("world_model", {}).get("action_dim", 4),
                target_xy=target_xy,
                obstacle_xy=obstacle_xy,
                layout_id=layout_id,
            )

            # env.step() 内部调用 execute_push 并返回 next_state
            next_state, reward, done, info = env.step(action)

            states_list.append(state)
            actions_list.append(action)
            next_states_list.append(next_state)
            total_steps += 1

            state = next_state
            if done:
                break

        pbar.update(1)
        if ep % 50 == 0:
            pbar.set_postfix({"total_steps": total_steps})

    pbar.close()
    env.close()

    # ---- 保存 ----
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    states_arr = np.array(states_list, dtype=np.float32)
    actions_arr = np.array(actions_list, dtype=np.float32)
    next_states_arr = np.array(next_states_list, dtype=np.float32)

    np.savez(
        args.output,
        states=states_arr,
        actions=actions_arr,
        next_states=next_states_arr,
    )

    # ---- 统计 ----
    print(f"\n保存完成: {total_steps} transitions from {args.episodes} episodes")
    print(f"  states:      {states_arr.shape}")
    print(f"  actions:     {actions_arr.shape}")
    print(f"  next_states: {next_states_arr.shape}")
    print(f"  output:      {args.output}")

    # 位移统计 (target + obstacle)
    t_disp = next_states_arr[:, 0:2] - states_arr[:, 0:2]
    t_dist = np.linalg.norm(t_disp, axis=1)
    o_disp = next_states_arr[:, 3:5] - states_arr[:, 3:5]
    o_dist = np.linalg.norm(o_disp, axis=1)
    print(f"\n数据统计:")
    print(f"  target   位移 (mm): mean={t_dist.mean():.1f}, std={t_dist.std():.1f}, "
          f"max={t_dist.max():.1f}")
    print(f"  obstacle 位移 (mm): mean={o_dist.mean():.1f}, std={o_dist.std():.1f}, "
          f"max={o_dist.max():.1f}")
    print(f"  state  mean (first 6):  {states_arr[:, :6].mean(axis=0)}")
    print(f"  action mean:             {actions_arr.mean(axis=0)}")

    # 质量检查: 任一物体发生位移即有效
    any_move = (t_dist > 0.5) | (o_dist > 0.5)
    moving_pct = any_move.mean() * 100
    print(f"  任一物体位移 (>0.5mm) 比例: {moving_pct:.1f}%")
    if moving_pct < 30:
        print("  WARNING: 有效位移比例偏低, 检查仿真/动作参数!")


if __name__ == "__main__":
    main()
