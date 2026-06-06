"""评估指标计算"""
import numpy as np
from typing import Dict, List


def compute_success_rate(results: List[bool]) -> float:
    """计算成功率"""
    if not results:
        return 0.0
    return sum(results) / len(results)


def compute_mean_planning_steps(steps: List[int]) -> float:
    """平均规划步数"""
    if not steps:
        return 0.0
    return np.mean(steps)


def compute_metrics(results: List[Dict]) -> Dict[str, float]:
    """汇总所有指标"""
    successes = [r.get("success", False) for r in results]
    steps = [r.get("total_steps", 0) for r in results]
    replans = [r.get("replan_count", 0) for r in results]

    return {
        "success_rate": compute_success_rate(successes),
        "mean_steps": compute_mean_planning_steps(steps),
        "mean_replans": np.mean(replans) if replans else 0.0,
        "total_trials": len(results),
    }
