"""可视化工具"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plot_push_trajectory(pred, real, goal, save_path):
    fig, ax = plt.subplots(figsize=(8, 6))
    if pred is not None: ax.plot(pred[:,0], pred[:,1], 'b--', label='Predicted')
    if real is not None: ax.plot(real[:,0], real[:,1], 'r-', label='Actual', lw=2)
    if pred is not None:
        ax.scatter(pred[0,0], pred[0,1], c='green', s=150, marker='o', label='Start')
    ax.scatter(goal[0], goal[1], c='lime', s=200, marker='*', label='Goal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('Push-Grasp Trajectory'); ax.legend()
    ax.grid(True, alpha=0.3); ax.set_aspect('equal')
    fig.savefig(save_path, dpi=150, bbox_inches='tight'); plt.close(fig)

def plot_success_rates(rates, labels, save_path):
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.bar(labels, rates, color=['green','blue','orange'])
    ax.set_ylabel('Success Rate'); ax.set_ylim(0, 1.1)
    ax.set_title('Task Success Rate by Layout')
    for i, r in enumerate(rates): ax.text(i, r+0.02, f'{r:.0%}', ha='center')
    fig.savefig(save_path, dpi=150, bbox_inches='tight'); plt.close(fig)

def plot_wm_error(pred_disp, real_disp, save_path):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(real_disp[:,0], pred_disp[:,0], alpha=0.5, s=10, label='X')
    ax.scatter(real_disp[:,1], pred_disp[:,1], alpha=0.5, s=10, label='Y')
    lims = [min(real_disp.min(), pred_disp.min()), max(real_disp.max(), pred_disp.max())]
    ax.plot(lims, lims, 'k--', alpha=0.5)
    ax.set_xlabel('Real displacement'); ax.set_ylabel('Predicted displacement')
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
    ax.set_title('World Model Prediction Error')
    fig.savefig(save_path, dpi=150, bbox_inches='tight'); plt.close(fig)
