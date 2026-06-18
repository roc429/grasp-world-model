#!/usr/bin/env python3
"""Quick test: env + planner."""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.simulation.env import PushGraspEnv
from src.utils.config import load_config
from src.planner.shooting_planner import ShootingPlanner

class DummyWM:
    def predict(self, s, a):
        ns = s.copy()
        ns[0] += a[3] * np.cos(a[2]) * 0.5
        ns[1] += a[3] * np.sin(a[2]) * 0.5
        gd = np.sqrt((ns[0]-ns[6])**2 + (ns[1]-ns[7])**2)
        return ns, 1.0/(1.0+gd*8)
    def predict_trajectory(self, s, seq):
        traj = np.zeros((len(seq)+1, len(s)), dtype=np.float32)
        scores = np.zeros(len(seq), dtype=np.float32)
        traj[0] = s; cur = s
        for t in range(len(seq)):
            ns, sc = self.predict(cur, seq[t])
            traj[t+1] = ns; scores[t] = sc; cur = ns
        return traj, scores

def main():
    print("Testing env...")
    config = load_config("config/default.yaml")
    env = PushGraspEnv(config)
    for lid in [1,2,3]:
        s = env.reset(layout_id=lid)
        print("  Layout {}: target=({:.3f},{:.3f})".format(lid, s[0], s[1]))
    print("  Env OK")

    print("Testing planner...")
    planner = ShootingPlanner(load_config("config/planner.yaml"))
    state = env.reset(layout_id=2)
    action, score = planner.plan(state, DummyWM())
    print("  Best: angle={:.2f} dist={:.3f} score={:.3f}".format(action[2], action[3], score))
    print("  Planner OK")

    print("Testing step...")
    ns, r, d, info = env.step(action)
    print("  After step: obj=({:.3f},{:.3f}) graspable={} done={}".format(ns[0], ns[1], info["graspable"], d))
    print("  Step OK")
    env.close()
    print("
ALL TESTS PASSED")

if __name__ == "__main__":
    main()
