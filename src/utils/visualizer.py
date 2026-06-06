"""可视化工具"""
import numpy as np
import matplotlib.pyplot as plt
from typing import List


def plot_trajectory_comparison(
    pred_traj: np.ndarray,
    real_traj: np.ndarray,
    save_path: str = None,
):
    """对比预测轨迹 vs 真实轨迹 (2D 俯视图)"""
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(pred_traj[:, 0], pred_traj[:, 1], "b--", label="Predicted", alpha=0.7)
    ax.plot(real_traj[:, 0], real_traj[:, 1], "r-", label="Actual", alpha=0.7)
    ax.scatter(pred_traj[0, 0], pred_traj[0, 1], c="green", s=100, marker="o", label="Start")
    ax.scatter(pred_traj[-1, 0], pred_traj[-1, 1], c="red", s=100, marker="x", label="End")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("Trajectory: Predicted vs Actual")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()


def plot_success_rate(
    success_rates: List[float],
    labels: List[str],
    save_path: str = None,
):
    """绘制成功率柱状图"""
    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(labels))
    bars = ax.bar(x, success_rates, width=0.5)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Success Rate")
    ax.set_ylim(0, 1.1)
    ax.set_title("Task Success Rate by Layout")

    for bar, rate in zip(bars, success_rates):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02,
                f"{rate:.1%}", ha="center", fontsize=10)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
