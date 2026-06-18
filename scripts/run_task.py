#!/usr/bin/env python3
"""Complete push-grasp task runner."""
import sys, os, argparse, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.simulation.env import PushGraspEnv
from src.utils.config import load_config
from src.utils.visualizer import plot_push_trajectory
from src.perception.detector import detect_objects
from src.world_model.inference import WorldModel
from src.planner.cem_planner import CEMPlanner
from src.planner.shooting_planner import ShootingPlanner


class HeuristicWM:
    def predict(self, s, a):
        sx, sy, ar, d = float(a[0]), float(a[1]), float(a[2]), float(a[3])
        ns = s.copy()
        ns[0] = sx + d * np.cos(ar)
        ns[1] = sy + d * np.sin(ar)
        gd = np.sqrt((ns[0] - ns[6])**2 + (ns[1] - ns[7])**2)
        score = 1.0 / (1.0 + gd * 8)
        if not (-0.05 < ns[0] < 0.40 and -0.20 < ns[1] < 0.20):
            score -= 100
        return ns, score

    def predict_trajectory(self, s, a_seq):
        H = len(a_seq)
        traj = np.zeros((H + 1, len(s)), dtype=np.float32)
        scores = np.zeros(H, dtype=np.float32)
        traj[0] = s
        cur = s
        for t in range(H):
            ns, sc = self.predict(cur, a_seq[t])
            traj[t + 1] = ns
            scores[t] = sc
            cur = ns
        return traj, scores


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--layout', type=int, default=2)
    parser.add_argument('--trials', type=int, default=10)
    parser.add_argument('--planner', type=str, default='cem')
    args = parser.parse_args()

    config = load_config('config/default.yaml')
    env = PushGraspEnv(config)

    wm_path = 'models/world_model/world_model.pt'
    if os.path.exists(wm_path):
        print('Loading trained world model:', wm_path)
        wm = WorldModel(wm_path, load_config('config/world_model.yaml'))
        wm_type = 'trained'
    else:
        print('Using heuristic world model (no trained model found)')
        wm = HeuristicWM()
        wm_type = 'heuristic'

    planner_cfg = load_config('config/planner.yaml')
    if args.planner == 'cem':
        planner = CEMPlanner(planner_cfg)
    else:
        planner = ShootingPlanner(planner_cfg)

    exp_tag = 'layout{}_{}_{}'.format(args.layout, args.planner, wm_type)
    exp_dir = os.path.join('experiments', exp_tag)
    os.makedirs(exp_dir, exist_ok=True)

    results = []
    for trial in range(args.trials):
        print('')
        print('=' * 50)
        print('Trial {}/{} | Layout {} | {}'.format(
            trial + 1, args.trials, args.layout, args.planner))
        print('=' * 50)

        state = env.reset(layout_id=args.layout)
        goal = env._goal_pos
        done = False
        step = 0
        replans = 0
        real_tr = [state[0:2].copy()]
        pred_tr = [state[0:2].copy()]

        while not done and step < 10:
            action, score = planner.plan(state, wm)
            replans += 1
            ns, reward, done, info = env.step(action)
            real_tr.append(ns[0:2].copy())
            ns_pred, _ = wm.predict(state, action)
            pred_tr.append(ns_pred[0:2].copy())

            print('  Step {}: angle={:.2f} dist={:.3f} score={:.3f} -> pos=({:.3f},{:.3f}) graspable={} done={}'.format(
                step, action[2], action[3], score, ns[0], ns[1], info['graspable'], done))
            state = ns
            step += 1

            if info['graspable'] and not done:
                print('  >>> Virtual grasp+place SUCCESS')
                done = True

        success = info.get('graspable', False)
        results.append({
            'trial': trial, 'success': success,
            'steps': step, 'replans': replans
        })
        plot_push_trajectory(
            np.array(pred_tr), np.array(real_tr), goal,
            os.path.join(exp_dir, 'trial_{}.png'.format(trial)))
        print('  Result: {} (steps={}, replans={})'.format(
            'SUCCESS' if success else 'FAILED', step, replans))

    rate = sum(r['success'] for r in results) / len(results)
    print('')
    print('=' * 50)
    print('FINAL Layout {}: Success {:.1%} ({}/{})'.format(
        args.layout, rate, sum(r['success'] for r in results), len(results)))
    print('Avg steps: {:.1f}'.format(
        np.mean([r['steps'] for r in results])))
    print('=' * 50)

    with open(os.path.join(exp_dir, 'results.json'), 'w') as f:
        json.dump({
            'success_rate': rate, 'wm_type': wm_type,
            'planner': args.planner, 'layout': args.layout,
            'trials': results
        }, f, indent=2)
    env.close()


if __name__ == '__main__':
    main()
