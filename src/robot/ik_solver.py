"""
Dobot Magician 逆运动学求解器 (4轴桌面机械臂)

DH 参数与几何结构:
  - J1: 基座旋转 (绕 Z 轴)，角度范围约 -90° ~ 90°
  - J2: 肩关节 (垂直面内)，驱动大臂
  - J3: 肘关节 (垂直面内)，驱动小臂
  - J4: 腕部旋转 (末端执行器朝向)

解耦策略:
  1. J1 = atan2(y, x) — 旋转基座对准目标
  2. 在 J1 确定的垂直平面内，解 J2, J3 作为 2 连杆平面机械臂
  3. J4 = r — 末端旋转角直接赋值

用法:
  from src.robot.ik_solver import solve_ik, solve_fk, check_workspace
  joints = solve_ik(x=200, y=0, z=50, r=0)  # 返回 (j1, j2, j3, j4) 单位度
"""

import numpy as np
from typing import Tuple, Optional
import logging

logger = logging.getLogger(__name__)

# ── Dobot Magician 近似 DH 参数 (mm, 度) ─────────────────
# 注: 精确参数需通过标定获得，以下为工程近似值

L1 = 135.0          # 大臂长度 (J2 → J3 距离, mm)
L2 = 147.0          # 小臂长度 (J3 → 末端, mm)
BASE_HEIGHT = 105.0  # 基座 J2 轴心高度 (mm, 相对桌面)
WRIST_OFFSET = 0.0   # 腕部偏移 (mm)，Dobot 通常为 0

# 关节限位 (度)
JOINT_LIMITS = {
    "j1": (-90.0, 90.0),
    "j2": (0.0, 85.0),       # 肩关节: 0°=竖直向下, 正值前倾
    "j3": (-120.0, 120.0),   # 肘关节: 0°=共线, 负=前弯, 正=后弯
    "j4": (-180.0, 180.0),
}

# 工作空间边界 (mm, 绝对坐标)
WORKSPACE = {
    "x": (50, 300),
    "y": (-150, 150),
    "z": (-50, 150),
}


# ── 正运动学 (FK) ────────────────────────────────────────

def solve_fk(j1_deg: float, j2_deg: float, j3_deg: float,
             j4_deg: float = 0.0) -> Tuple[float, float, float, float]:
    """
    正运动学: 关节角度 → 末端位姿 (x, y, z, r)。

    Args:
        j1_deg: J1 基座旋转角 (度)
        j2_deg: J2 肩关节角 (度, 0°=竖直向上)
        j3_deg: J3 肘关节角 (度)
        j4_deg: J4 腕部旋转角 (度)

    Returns:
        (x, y, z, r) 末端位姿 (mm, mm, mm, 度)
    """
    j1 = np.radians(j1_deg)
    j2 = np.radians(j2_deg)
    j3 = np.radians(j3_deg)

    # J2 和 J3 在垂直平面内构成 2 连杆
    # J2 相对于基座，J3 相对于 J2
    theta_arm = j2            # 大臂与竖直方向的夹角
    theta_elbow = j2 + j3     # 小臂与竖直方向的夹角

    # 末端在基座坐标系中的位置
    r_eff = L1 * np.sin(theta_arm) + L2 * np.sin(theta_elbow)  # 水平距离
    z_eff = BASE_HEIGHT - (L1 * np.cos(theta_arm) + L2 * np.cos(theta_elbow))  # 高度

    # J1 旋转
    x = r_eff * np.cos(j1)
    y = r_eff * np.sin(j1)
    z = z_eff

    return (x, y, z, j4_deg)


# ── 逆运动学 (IK) ────────────────────────────────────────

def solve_ik(x: float, y: float, z: float, r: float = 0.0,
             config: str = "elbow_down") -> Optional[Tuple[float, float, float, float]]:
    """
    逆运动学: 末端位姿 → 关节角度。

    解耦策略:
      - J1: 由 x,y 直接求解
      - J2,J3: 在垂直平面内解 2 连杆 IK
      - J4: 直接赋值为 r

    Args:
        x, y, z: 末端目标位置 (mm)
        r: 末端旋转角 (度)
        config: "elbow_down" (默认) 或 "elbow_up"

    Returns:
        (j1, j2, j3, j4) 关节角度 (度)，无解返回 None
    """
    # 1. 工作空间检查
    if not check_workspace(x, y, z):
        logger.warning(f"IK: 目标 ({x:.0f},{y:.0f},{z:.0f}) 超出工作空间")
        return None

    # 2. J1: 基座旋转
    j1 = np.degrees(np.arctan2(y, x))
    if not _in_limit("j1", j1):
        logger.warning(f"IK: J1={j1:.1f}° 超出限位")
        return None

    # 3. 在 J1 的垂直平面内，目标投影到 (r_horiz, z_from_shoulder)
    r_horiz = np.sqrt(x**2 + y**2)      # 水平距离

    # 目标在 J2 坐标系中: z_from_shoulder = BASE_HEIGHT - z
    # (正值=J2 下方/桌面方向, 即 arm 向下伸)
    z_from_shoulder = BASE_HEIGHT - z

    # 2 连杆平面 IK: 求解 J2, J3
    result = _solve_2link_ik(r_horiz, z_from_shoulder, config)
    if result is None:
        logger.warning(f"IK: 2连杆无解 (r={r_horiz:.0f}, z_down={z_from_shoulder:.0f})")
        return None

    j2, j3 = result

    # 4. J4: 末端旋转角
    j4 = r

    # 5. 验证
    x_fk, y_fk, z_fk, _ = solve_fk(j1, j2, j3, j4)
    pos_err = np.sqrt((x - x_fk)**2 + (y - y_fk)**2 + (z - z_fk)**2)
    if pos_err > 5.0:
        logger.warning(f"IK: FK 验证偏差较大 ({pos_err:.1f}mm)")

    return (j1, j2, j3, j4)


def _solve_2link_ik(r: float, z_down: float, config: str = "elbow_down"
                    ) -> Optional[Tuple[float, float]]:
    """
    2 连杆平面 IK: (r, z_down) → (j2_deg, j3_deg)。

    坐标系:
      - r:   水平距离 (mm)
      - z_down: J2 轴心到目标的垂直距离 (mm, 正=向下/朝桌面)
      - j2: 从竖直向下方向算起的角度 (度, 0=竖直向下, 正=前倾)
      - j3: 肘关节相对角度 (度, 0=大小臂共线)

    正向映射:
      r = L1·sin(j2) + L2·sin(j2+j3)
      z_down = L1·cos(j2) + L2·cos(j2+j3)

    使用余弦定理求解。

    Returns:
        (j2_deg, j3_deg) 或 None
    """
    d2 = r**2 + z_down**2     # 目标距离平方
    d = np.sqrt(d2)

    # 超出臂展范围
    if d > L1 + L2 + 2.0:
        return None
    if d < abs(L1 - L2) - 2.0:
        return None

    # 余弦定理求 J3
    cos_j3 = (d2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_j3 = np.clip(cos_j3, -1.0, 1.0)
    j3_abs = np.arccos(cos_j3)  # 0 ~ π

    if config == "elbow_up":
        j3_deg = np.degrees(j3_abs)
    else:
        j3_deg = -np.degrees(j3_abs)

    j3_rad = np.radians(j3_deg)

    # 求解 J2:
    # j2 = atan2(r, z_down) - atan2(L2·sin(j3), L1 + L2·cos(j3))
    alpha = np.arctan2(r, z_down)  # 目标方向角 (从竖直向下算)
    beta = np.arctan2(L2 * np.sin(j3_rad), L1 + L2 * np.cos(j3_rad))
    j2_rad = alpha - beta
    j2_deg = np.degrees(j2_rad)

    # 检查限位，超标则尝试另一配置
    if not _in_limit("j2", j2_deg) or not _in_limit("j3", j3_deg):
        if config == "elbow_down":
            return _solve_2link_ik(r, z_down, "elbow_up")
        return None

    return (j2_deg, j3_deg)


# ── 工作空间与限位 ───────────────────────────────────────

def check_workspace(x: float, y: float, z: float) -> bool:
    """检查目标位姿是否在机械臂工作空间内"""
    if not (WORKSPACE["x"][0] <= x <= WORKSPACE["x"][1]):
        return False
    if not (WORKSPACE["y"][0] <= y <= WORKSPACE["y"][1]):
        return False
    if not (WORKSPACE["z"][0] <= z <= WORKSPACE["z"][1]):
        return False

    # 额外检查: 水平距离不应超过臂展
    r = np.sqrt(x**2 + y**2)
    if r > L1 + L2 + 10:
        return False

    return True


def _in_limit(joint: str, angle_deg: float) -> bool:
    """检查关节角度是否在限位内"""
    limits = JOINT_LIMITS.get(joint)
    if limits is None:
        return True
    lo, hi = limits
    return lo <= angle_deg <= hi


def get_joint_limits() -> dict:
    """返回关节限位字典"""
    return JOINT_LIMITS.copy()


def get_link_lengths() -> dict:
    """返回连杆长度参数"""
    return {"L1": L1, "L2": L2, "base_height": BASE_HEIGHT}


# ── 便捷接口 (兼容旧 API) ─────────────────────────────────

def solve_ik_dobot(x: float, y: float, z: float, r: float = 0.0
                   ) -> Tuple[float, float, float, float]:
    """
    兼容旧 API: 求解 IK 并返回通过检查的位姿。
    如果 IK 无解则抛出 ValueError。
    """
    result = solve_ik(x, y, z, r)
    if result is None:
        raise ValueError(
            f"IK 无解: target=({x:.0f},{y:.0f},{z:.0f}), "
            f"workspace: X={WORKSPACE['x']}, Y={WORKSPACE['y']}, Z={WORKSPACE['z']}"
        )
    return result


# ── 自测 ──────────────────────────────────────────────────

def _self_test():
    """IK 自测: 随机采样验证 FK→IK→FK 一致性"""
    print("IK 求解器自测...")
    np.random.seed(42)

    errors = []
    for _ in range(100):
        # 随机生成合法关节角度
        j1 = np.random.uniform(*JOINT_LIMITS["j1"])
        j2 = np.random.uniform(*JOINT_LIMITS["j2"])
        j3 = np.random.uniform(*JOINT_LIMITS["j3"])
        j4 = np.random.uniform(*JOINT_LIMITS["j4"])

        # FK
        x, y, z, r = solve_fk(j1, j2, j3, j4)

        # 跳过超出工作空间的结果
        if not check_workspace(x, y, z):
            continue

        # IK
        result = solve_ik(x, y, z, r)
        if result is None:
            print(f"  FAIL: FK→IK 无解: j=({j1:.1f},{j2:.1f},{j3:.1f}) → pos=({x:.0f},{y:.0f},{z:.0f})")
            continue

        j1_ik, j2_ik, j3_ik, j4_ik = result

        # FK again
        x2, y2, z2, r2 = solve_fk(j1_ik, j2_ik, j3_ik, j4_ik)

        err = np.sqrt((x - x2)**2 + (y - y2)**2 + (z - z2)**2)
        errors.append(err)

        if err > 5.0:
            print(f"  WARN: pos_err={err:.1f}mm  j_in=({j1:.0f},{j2:.0f},{j3:.0f}) j_ik=({j1_ik:.0f},{j2_ik:.0f},{j3_ik:.0f})")

    if errors:
        print(f"  结果: mean_err={np.mean(errors):.2f}mm  max_err={np.max(errors):.1f}mm  n={len(errors)}")
    else:
        print("  无有效测试点 (所有 FK 结果超出工作空间)")

    return len(errors) > 0 and np.mean(errors) < 5.0


if __name__ == "__main__":
    _self_test()
