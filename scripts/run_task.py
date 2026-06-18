#!/usr/bin/env python3
"""完整推-抓任务执行脚本 — 适配滕涛 SimulationEnv"""
import sys, os, argparse, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.simulation.env import SimulationEnv
from src.utils.config import load_config
from src.utils.visualizer import plot_push_trajectory, plot_success_rates


# ============================================================
# 简化启发式世界模型 (无需训练, 立即可用)
# ============================================================
class HeuristicWorldModel:
    """
    基于物理常识的启发式世界模型:
    - 预测: 物体沿推的方向直线移动
    - 评分: 物体离场景放置区越近分数越高, 被障碍物遮挡时减分
    """

    def __init__(self):
        self.state_dim = 10

    def predict(self, state: np.ndarray, action: np.ndarray):
        """
        Args:
            state:  [target_x, target_y, target_z, obs_x, obs_y, obs_z,
                     ee_x, ee_y, ee_z, gripper]  单位 mm
            action: [start_x, start_y, direction_angle, distance]  mm, rad, mm
        Returns:
            next_state, score
        """
        sx, sy = float(action[0]), float(action[1])
        angle = float(action[2])
        dist = float(action[3])
        ex = sx + dist * np.cos(angle)
        ey = sy + dist * np.sin(angle)

        ns = state.copy()
        ns[0] = ex  # 预测目标物体移动到推的终点
        ns[1] = ey

        # 评分: 物体在工作空间内 + 不与障碍物重叠
        score = 0.0
        ws_ok = (0 < ex < 400 and -150 < ey < 150)
        if ws_ok:
            score += 5.0
        else:
            score -= 100.0

        # 与障碍物距离 (如果障碍物存在)
        ox, oy = ns[3], ns[4]
        if ox > 0 and oy > -200:  # 障碍物在场景内
            obs_dist = np.sqrt((ex - ox)**2 + (ey - oy)**2)
            if obs_dist < 50:
                score -= 20.0  # 太靠近障碍物

        return ns, score

    def predict_trajectory(self, state, action_sequence):
        H = len(action_sequence)
        traj = np.zeros((H + 1, len(state)), dtype=np.float32)
        scores = np.zeros(H, dtype=np.float32)
        traj[0] = state
        cur = state
        for t in range(H):
            ns, sc = self.predict(cur, action_sequence[t])
            traj[t + 1] = ns
            scores[t] = sc
            cur = ns
        return traj, scores


# ============================================================
# 主函数
# ============================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--layout', type=int, default=2)
    parser.add_argument('--trials', type=int, default=10)
    parser.add_argument('--planner', type=str, default='cem')
    args = parser.parse_args()

    config = load_config('config/default.yaml')
    env = SimulationEnv(config)

    # 世界模型: 优先尝试加载训练好的, 不行就用启发式
    wm_path = 'models/world_model/world_model.pt'
    if os.path.exists(wm_path):
        print('[WM] Loading trained world model:', wm_path)
        from src.world_model.inference import WorldModel
        wm = WorldModel(wm_path, load_config('config/world_model.yaml'))
        wm_type = 'trained_mlp'
    else:
        print('[WM] Using heuristic world model (no trained model found)')
        wm = HeuristicWorldModel()
        wm_type = 'heuristic'

    # 规划器
    from src.planner.cem_planner import CEMPlanner
    from src.planner.shooting_planner import ShootingPlanner
    planner_cfg = load_config('config/planner.yaml')
    planner = CEMPlanner(planner_cfg) if args.planner == 'cem' else ShootingPlanner(planner_cfg)

    exp_dir = 'experiments/layout{}_{}_{}'.format(args.layout, args.planner, wm_type)
    os.makedirs(exp_dir, exist_ok=True)

    results = []
    for trial in range(args.trials):
        print('')
        print('=' * 55)
        print('Trial {}/{} | Layout {} | {} | WM={}'.format(
            trial + 1, args.trials, args.layout, args.planner, wm_type))
        print('=' * 55)

        state = env.reset(layout_id=args.layout)
        objs = env.get_objects_state()
        target_start = objs['target'].copy()
        placement_xy = env._get_placement_xy_mm()
        print('Target start: ({:.1f}, {:.1f}) mm'.format(target_start[0], target_start[1]))

        done = False
        step = 0
        real_traj = [target_start[:2].copy()]

        while not done and step < 5:
            # 1. 规划推动作
            action, score = planner.plan(state, wm)
            print('  Step {}: angle={:.1f}deg dist={:.1f}mm score={:.2f}'.format(
                step, np.degrees(action[2]), action[3], score))

            # 2. 执行推动作 (用滕涛的 env 内置方法)
            env.execute_push(
                start_xy_mm=np.array([action[0], action[1]]),
                direction_angle=float(action[2]),
                distance_mm=float(action[3]),
                velocity=80.0,
            )

            # 3. 获取新状态
            state = env._build_state()
            objs = env.get_objects_state()
            real_traj.append(objs['target'][:2].copy())
            step += 1

            # 4. 检查是否可抓取
            graspable = _check_graspable(env, state)
            print('    Target now: ({:.1f}, {:.1f}) mm, graspable={}'.format(
                objs['target'][0], objs['target'][1], graspable))

            if graspable:
                print('  -> Object reachable! Executing grasp+place...')
                env.execute_grasp(
                    obj_xy_mm=objs['target'][:2],
                    obj_z_mm=objs['target'][2],
                )
                env.execute_place(
                    target_xy_mm=placement_xy,
                    target_z_mm=15.0,
                )
                done = True
            elif _object_lost(state):
                print('  -> Object lost (out of workspace)')
                done = True

        success = done and not _object_lost(state)
        results.append({
            'trial': trial, 'success': success,
            'steps': step, 'planner': args.planner, 'wm': wm_type,
        })

        # 可视化
        plot_push_trajectory(
            np.array(real_traj), np.array(real_traj),
            placement_xy / 1000.0,  # visualizer 用米
            '{}/trial_{}.png'.format(exp_dir, trial),
        )
        print('  Result: {} ({} steps)'.format(
            'SUCCESS' if success else 'FAILED', step))

    # ── 汇总 ──
    success_count = sum(r['success'] for r in results)
    rate = success_count / len(results)
    print('')
    print('=' * 55)
    print('Layout {} FINAL: {:.1%} ({}/{})'.format(
        args.layout, rate, success_count, len(results)))
    print('=' * 55)

    with open('{}/results.json'.format(exp_dir), 'w') as f:
        json.dump({'success_rate': rate, 'wm': wm_type,
                   'planner': args.planner, 'layout': args.layout,
                   'trials': results}, f, indent=2, ensure_ascii=False)

    env.close()


def _check_graspable(env, state) -> bool:
    """检查目标是否可被机械臂抓取"""
    tx, ty = state[0], state[1]
    # 简单规则: 在工作空间前方中心区域
    if 80 < tx < 350 and -80 < ty < 80:
        return True
    return False


def _object_lost(state) -> bool:
    tx, ty = state[0], state[1]
    return not (-50 < tx < 450 and -200 < ty < 200)


if __name__ == '__main__':
    main()
