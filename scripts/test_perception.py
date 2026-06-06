"""
W3/W6 感知与 IK 验证脚本

用法:
    python scripts/test_perception.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from src.utils.config import load_config
from src.simulation.env import SimulationEnv
from src.simulation.scene_builder import load_scene_config
from src.robot.ik_solver import solve_ik, solve_fk, get_link_lengths, get_joint_limits
from src.perception.camera import SimCamera


def test_ik():
    """测试 IK 求解器"""
    print("=" * 55)
    print("IK 求解器验证")
    print("=" * 55)

    links = get_link_lengths()
    limits = get_joint_limits()
    print(f"  连杆参数: L1={links['L1']}mm, L2={links['L2']}mm, base={links['base_height']}mm")
    print(f"  关节限位: J1={limits['j1']}, J2={limits['j2']}, J3={limits['j3']}")

    # 测试点（均为 Dobot 实际可达区域）
    test_cases = [
        {"pos": (200, 0, 10), "desc": "正前方桌面"},
        {"pos": (180, -60, 30), "desc": "左前方稍高"},
        {"pos": (250, 80, 5), "desc": "右前方近桌面"},
        {"pos": (200, -100, 50), "desc": "左前方中高"},
        {"pos": (150, 50, 10), "desc": "右前方近端"},
        {"pos": (240, 0, 5), "desc": "最远低处"},
        {"pos": (220, 80, 20), "desc": "右前中距"},
    ]

    all_pass = True
    for tc in test_cases:
        x, y, z = tc["pos"]
        result = solve_ik(x, y, z, r=0)
        if result is None:
            print(f"  FAIL: {tc['desc']} ({x},{y},{z}) — 无解")
            all_pass = False
            continue
        j1, j2, j3, j4 = result
        x_fk, y_fk, z_fk, _ = solve_fk(j1, j2, j3, j4)
        err = np.sqrt((x-x_fk)**2 + (y-y_fk)**2 + (z-z_fk)**2)
        status = "OK" if err < 1.0 else f"ERR={err:.1f}mm"
        print(f"  {status}: {tc['desc']} target=({x},{y},{z}) -> "
              f"J=({j1:.0f},{j2:.0f},{j3:.0f}) -> FK=({x_fk:.0f},{y_fk:.0f},{z_fk:.0f})")
        if err >= 1.0:
            all_pass = False

    # 边界测试
    print("\n  边界测试:")
    bad = solve_ik(400, 0, 10)  # 超出范围
    print(f"  超范围 X=400: {'正确返回None' if bad is None else 'FAIL'}")

    bad2 = solve_ik(200, 200, 10)  # Y 超范围
    print(f"  超范围 Y=200: {'正确返回None' if bad2 is None else 'FAIL'}")

    print(f"\n  IK 结果: {'ALL PASS' if all_pass else 'SOME FAILED'}")
    return all_pass


def test_camera():
    """测试仿真相机渲染"""
    print("\n" + "=" * 55)
    print("仿真相机验证")
    print("=" * 55)

    config = load_config("config/default.yaml")
    env = SimulationEnv(config)
    env.reset(layout_id=2)

    cam = SimCamera(env, width=640, height=480)

    # 1. 获取 RGB-D
    print("  [1/3] 渲染 RGB-D...")
    try:
        rgb, depth = cam.get_rgbd()
        print(f"       RGB:  {rgb.shape} {rgb.dtype}  range=[{rgb.min()},{rgb.max()}]")
        print(f"       Depth: {depth.shape} {depth.dtype}  range=[{depth.min():.0f},{depth.max():.0f}]mm")
    except Exception as e:
        print(f"       FAIL: {e}")
        cam.close()
        env.close()
        return False

    # 2. 检查图像是否有效
    print("  [2/3] 检查图像有效性...")
    rgb_ok = rgb.shape == (480, 640, 3) and rgb.max() > 0
    depth_ok = depth.shape == (480, 640) and depth.max() > 0
    print(f"       RGB valid: {rgb_ok}, Depth valid: {depth_ok}")

    # 3. 内参
    print("  [3/3] 相机内参...")
    K = cam.get_intrinsics()
    print(f"       fx={K['fx']:.1f}, fy={K['fy']:.1f}, "
          f"cx={K['cx']:.1f}, cy={K['cy']:.1f}")

    cam.close()
    env.close()
    return rgb_ok and depth_ok


def main():
    ik_ok = test_ik()
    cam_ok = test_camera()

    print("\n" + "=" * 55)
    print(f"IK: {'PASS' if ik_ok else 'FAIL'}  |  Camera: {'PASS' if cam_ok else 'FAIL'}")
    print("=" * 55)


if __name__ == "__main__":
    main()
