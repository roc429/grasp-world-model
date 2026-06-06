"""逆运动学求解器 (简化为 Dobot 4轴直接控制, 无需复杂IK)"""

import numpy as np
from typing import Tuple


def solve_ik_dobot(x: float, y: float, z: float, r: float = 0.0
                   ) -> Tuple[float, ...]:
    """
    Dobot Magician 的简单 IK: 笛卡尔坐标直接对应末端位姿。
    Dobot 内部自动处理关节角度计算。这里只做工作空间检查。

    Returns:
        (x, y, z, r) 通过检查的位姿
    """
    # 工作空间检查 (mm)
    if not (50 <= x <= 300):
        raise ValueError(f"X={x} out of workspace [50, 300]")
    if not (-150 <= y <= 150):
        raise ValueError(f"Y={y} out of workspace [-150, 150]")
    if not (-50 <= z <= 150):
        raise ValueError(f"Z={z} out of workspace [-50, 150]")

    return (x, y, z, r)
