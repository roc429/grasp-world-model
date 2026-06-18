#!/usr/bin/env python3
"""Validate world model prediction accuracy on test data."""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.world_model.inference import WorldModel
from src.utils.config import load_config
from src.simulation.env import SimulationEnv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    config = load_config('config/world_model.yaml')
    wm = WorldModel('models/world_model/world_model.pt', config)
    env = SimulationEnv(load_config('config/default.yaml'))
    pred_disp, real_disp = [], []

    for layout in [1, 2, 3]:
        for _ in range(100):
            state = env.reset(layout_id=layout)
            obj = env.get_objects_state()['target']
            action = np.array([
                obj[0] + np.random.uniform(-20, 20),
                obj[1] + np.random.uniform(-20, 20),
                np.random.uniform(0, 2*np.pi),
                np.random.uniform(10, 80)
            ], dtype=np.float32)
            pred_state, _ = wm.predict(state, action)
            env.execute_push(obj[:2], float(action[2]), float(action[3]), velocity=80.0)
            real_state = env._build_state()
            pred_disp.append(pred_state[0:2] - state[0:2])
            real_disp.append(real_state[0:2] - state[0:2])

    pred = np.array(pred_disp)
    real = np.array(real_disp)
    mse_x = np.mean((pred[:,0] - real[:,0])**2)
    mse_y = np.mean((pred[:,1] - real[:,1])**2)
    print('RMSE X: {:.2f} mm'.format(np.sqrt(mse_x)))
    print('RMSE Y: {:.2f} mm'.format(np.sqrt(mse_y)))

    os.makedirs('experiments', exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, dim, rmse, i in zip(axes, ['X', 'Y'], [np.sqrt(mse_x), np.sqrt(mse_y)], [0, 1]):
        ax.scatter(real[:,i], pred[:,i], alpha=0.5, s=5)
        ax.set_xlabel('Real {} displacement (mm)'.format(dim))
        ax.set_ylabel('Predicted {} displacement (mm)'.format(dim))
        ax.set_title('{} Prediction (RMSE={:.1f}mm)'.format(dim, rmse))
        lim = max(abs(real[:,i]).max(), abs(pred[:,i]).max()) * 1.2
        ax.plot([-lim, lim], [-lim, lim], 'k--', alpha=0.5)
        ax.set_aspect('equal')
    plt.tight_layout()
    plt.savefig('experiments/world_model_validation.png', dpi=150)
    print('Saved: experiments/world_model_validation.png')
    env.close()

if __name__ == '__main__':
    main()
