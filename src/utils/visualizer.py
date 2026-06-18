"""可视化工具"""
import os
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict, Optional


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


def plot_scene_topview(
    objects: Dict[str, np.ndarray],
    placement_xy: Optional[tuple] = None,
    ee_pos: Optional[tuple] = None,
    workspace: Optional[dict] = None,
    save_path: str = None,
    title: str = "Scene Top-Down View",
):
    """
    绘制场景俯视图（2D），用于调试和可视化。

    用法:
        objs = env.get_objects_state()
        plot_scene_topview(objs, placement_xy=(200, -100),
                          ee_pos=(ee[:2]), save_path="scene.png")
    """
    fig, ax = plt.subplots(figsize=(8, 8))

    if workspace:
        x_min, x_max = workspace["x"]
        y_min, y_max = workspace["y"]
        ax.plot([x_min, x_max, x_max, x_min, x_min],
                [y_min, y_min, y_max, y_max, y_min],
                "k--", linewidth=1, alpha=0.3, label="Workspace")

    target = objects.get("target")
    if target is not None:
        ax.scatter(target[0], target[1], c="red", s=250, marker="s",
                   edgecolors="darkred", linewidth=1.5, zorder=5, label="Target")

    obstacle = objects.get("obstacle")
    if obstacle is not None:
        ax.scatter(obstacle[0], obstacle[1], c="blue", s=200, marker="s",
                   edgecolors="darkblue", linewidth=1.5, zorder=5, label="Obstacle")

    if ee_pos is not None:
        ax.scatter(ee_pos[0], ee_pos[1], c="gray", s=180, marker="o",
                   edgecolors="black", linewidth=1, zorder=6, label="End Effector")

    if placement_xy is not None:
        circle = plt.Circle(placement_xy, 30, color="green", fill=False,
                            linestyle="--", linewidth=1.5, alpha=0.6,
                            label="Placement Zone")
        ax.add_patch(circle)

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    if workspace:
        ax.set_xlim(workspace["x"][0] - 20, workspace["x"][1] + 20)
        ax.set_ylim(workspace["y"][0] - 20, workspace["y"][1] + 20)

    if save_path:
        os.makedirs(os.path.dirname(save_path) if os.path.dirname(save_path) else ".", exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  [Viz] Saved: {save_path}")
    plt.close()
