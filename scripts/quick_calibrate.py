"""
快速标定: 机械臂末端点 → 像素坐标 → 世界坐标映射
不用标定板，机械臂本身就是标定点。

方法: 移动机械臂到 6+ 已知位姿，在相机画面中记录末端像素位置，
     解算 2D 仿射变换 (像素 → mm)
"""

import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2
import json

N_POINTS = 8  # 标定点数

# 预定义机械臂位姿 (x, y, z, r) — z 固定使得末端贴近桌面
POSES = [
    (180, -80, 10, 0),
    (180, 80, 10, 0),
    (250, -80, 10, 0),
    (250, 80, 10, 0),
    (215, 0, 10, 0),
    (150, 0, 10, 0),
    (200, -100, 10, 0),
    (200, 100, 10, 0),
]

Z_SAFE = 60.0


def find_ee_tip_auto(gray):
    """
    自动找末端: 找图像中最亮的圆形/点。
    如果末端有反光标记更好。
    返回 (cx, cy) 像素坐标 或 None
    """
    # 二值化找亮点
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    # 找最大的亮区域
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    return np.array([M["m10"]/M["m00"], M["m01"]/M["m00"]])


def main():
    print("=" * 50)
    print("快速标定: 机械臂末端 → 像素→世界映射")
    print("=" * 50)

    # ── 连接 ──
    from src.robot.dobot_arm import DobotArm
    from src.robot.base_arm import Pose
    from src.perception.hikvision_camera import HikvisionCamera

    arm = DobotArm()
    if not arm.connect("COM3"):
        print("机械臂失败"); return

    cam = HikvisionCamera()
    if not cam.open(0):
        print("相机失败"); arm.disconnect(); return
    print(f"相机: {cam.resolution}")

    # ── 采集 ──
    print(f"\n采集 {len(POSES)} 个点...")
    pixel_pts = []
    world_pts = []

    for i, (wx, wy, z, r) in enumerate(POSES):
        print(f"\n[{i+1}] 移动到世界 ({wx:.0f}, {wy:.0f}, {z:.0f})")

        arm.move_to_pose(Pose(wx, wy, Z_SAFE, r))
        time.sleep(0.2)
        arm.move_to_pose(Pose(wx, wy, z, r))
        time.sleep(0.5)

        frame = cam.grab(timeout_ms=2000)
        if frame is None:
            print("  采图失败")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        pixel = find_ee_tip_auto(gray)

        if pixel is not None:
            print(f"  像素: ({pixel[0]:.0f}, {pixel[1]:.0f})")
            world_pts.append([wx, wy])
            pixel_pts.append(pixel)
            cv2.circle(frame, (int(pixel[0]), int(pixel[1])), 15, (0,255,0), 3)
        else:
            print("  未找到末端亮点! 请确认:")
            print("  1) 末端在相机视野内")
            print("  2) 末端有反光标记 or 浅色物体")
            frame2 = frame.copy()
        cv2.circle(frame2, (frame2.shape[1]//2, frame2.shape[0]//2), 20, (255,0,0), 3)
        cv2.imwrite(f"experiments/calib_pt_{i+1:02d}.png", frame2[:,:,::-1])

    n = len(world_pts)
    print(f"\n成功采集 {n} 个点")

    if n < 4:
        print("至少需要 4 个点!"); cam.close(); arm.disconnect(); return

    # ── 解算仿射变换 ──
    pixel_pts = np.array(pixel_pts, dtype=np.float32)
    world_pts = np.array(world_pts, dtype=np.float32)

    # 2D 仿射: world = A @ pixel + b
    # 用最小二乘: [pixel, 1] @ [A; b] = world
    ones = np.ones((n, 1))
    X = np.hstack([pixel_pts, ones])
    M, _, _, _ = np.linalg.lstsq(X, world_pts, rcond=None)
    M = M.T  # (2, 3) affine matrix
    print(f"\n仿射矩阵 M (2x3):\n{M}")

    # 计算误差
    predicted = X @ M.T
    errors = np.linalg.norm(predicted - world_pts, axis=1)
    print(f"\n标定误差: mean={errors.mean():.1f}mm, max={errors.max():.1f}mm")

    # 保存
    result = {
        "type": "affine_2d",
        "matrix": M.tolist(),
        "n_points": n,
        "error_mean_mm": float(errors.mean()),
        "error_max_mm": float(errors.max()),
        "pixel_pts": pixel_pts.tolist(),
        "world_pts": world_pts.tolist(),
    }
    with open("config/quick_calib.json", "w") as f:
        json.dump(result, f, indent=2)
    print("已保存 config/quick_calib.json")

    # ── 验证: 输入像素，输出世界坐标 ──
    print("\n验证: 输入像素坐标 (或按 Enter 跳过)")
    while True:
        try:
            inp = input("像素 (cx cy): ").strip()
            if not inp:
                break
            cx, cy = map(float, inp.split())
            wx, wy = M @ np.array([cx, cy, 1.0])
            print(f"  世界坐标: ({wx:.1f}, {wy:.1f}) mm")
        except (EOFError, KeyboardInterrupt):
            break
        except:
            break

    arm.move_to_pose(Pose(200, 0, Z_SAFE, 0))
    cam.close()
    arm.disconnect()
    print("完成!")


if __name__ == "__main__":
    main()
