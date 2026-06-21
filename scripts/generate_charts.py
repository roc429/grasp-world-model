#!/usr/bin/env python3
"""生成实验图表: 消融成功率柱状图, loss 曲线等

用法:
    python scripts/generate_charts.py
"""

import sys, os, json, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def generate_ablation_chart(results_path="experiments/ablation_results.json",
                             output_path="experiments/ablation_success_rate.png"):
    """生成消融实验成功率柱状图"""
    if not os.path.exists(results_path):
        print(f"WARNING: {results_path} not found, skipping ablation chart")
        return

    with open(results_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    layouts = list(data.keys())
    heuristic_rates = [data[k]["heuristic"]["success_rate"] * 100 for k in layouts]
    wm_rates = [data[k]["world_model"]["success_rate"] * 100 for k in layouts]

    fig, ax = plt.subplots(figsize=(10, 6))
    x = np.arange(len(layouts))
    width = 0.35

    bars1 = ax.bar(x - width/2, heuristic_rates, width, label="Heuristic", color="#FF9800")
    bars2 = ax.bar(x + width/2, wm_rates, width, label="World Model", color="#2196F3")

    ax.set_xlabel("Layout")
    ax.set_ylabel("Success Rate (%)")
    ax.set_title("Ablation: Heuristic vs World Model")
    ax.set_xticks(x)
    ax.set_xticklabels([l.replace("layout_", "Layout ") for l in layouts])
    ax.legend()
    ax.set_ylim(0, 110)
    ax.grid(True, alpha=0.3, axis="y")

    for bar in bars1:
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f"{bar.get_height():.0f}%", ha="center", va="bottom", fontsize=10)
    for bar in bars2:
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f"{bar.get_height():.0f}%", ha="center", va="bottom", fontsize=10)

    plt.tight_layout()
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"Saved: {output_path}")


def generate_loss_chart(output_path="experiments/loss_curves.png"):
    """尝试从 TensorBoard 提取 loss 曲线"""
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

        fig, ax = plt.subplots(figsize=(10, 6))
        found_any = False

        for name, logdir, color in [
            ("MLP", "models/world_model/mlp/tensorboard", "#2196F3"),
            ("GRU", "models/world_model/gru/tensorboard", "#4CAF50"),
        ]:
            for event_file in glob.glob(f"{logdir}/**/events.out.tfevents.*", recursive=True):
                ea = EventAccumulator(event_file)
                ea.Reload()
                for tag, ls in [("Loss/train", "-"), ("Loss/val", "--")]:
                    if tag in ea.Tags().get("scalars", {}):
                        events = ea.Scalars(tag)
                        steps = [e.step for e in events]
                        vals = [e.value for e in events]
                        ax.plot(steps, vals, color=color, linestyle=ls,
                                label=f"{name} {tag.split('/')[-1]}", alpha=0.7)
                        found_any = True

        if found_any:
            ax.set_xlabel("Epoch")
            ax.set_ylabel("MSE Loss")
            ax.set_title("Training Curves: MLP vs GRU")
            ax.legend()
            ax.grid(True, alpha=0.3)
            plt.tight_layout()
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            plt.savefig(output_path, dpi=150)
            plt.close()
            print(f"Saved: {output_path}")
        else:
            print("No TensorBoard events found, skipping loss chart")
    except Exception as e:
        print(f"Loss chart generation failed: {e}")


if __name__ == "__main__":
    generate_ablation_chart()
    generate_loss_chart()
    print("Done! Check experiments/ for output files.")
