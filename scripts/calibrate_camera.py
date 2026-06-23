"""
手眼标定采集脚本 (Eye-to-Hand)
1. 打印桌面 calib_pattern.png，贴在机械臂末端
2. 运行此脚本，自动采集 N 组 (机械臂位姿, 标定板图像)
3. 自动计算手眼矩阵并保存
"""

import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2
import json

from src.robot.dobot_arm import DobotArm
from src.robot.base_arm import Pose

# ── 配置 ──────────────────────────────────────────
SQUARE_SIZE_MM = 20.0        # 棋盘格实际方格边长
CHECKERBOARD = (11, 8)       # 内角点数 (比格数少1)
N_SAMPLES = 15               # 采集组数
Z_SAFE = 80.0                # 安全移动高度

# 相机内参（默认值，标定后可更新）
K = np.array([
    [554.3, 0, 320.0],
    [0, 554.3, 240.0],
    [0, 0, 1]
], dtype=np.float64)
DIST = np.zeros((5,), dtype=np.float64)

# ── 标定板 3D 点 ──────────────────────────────────
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

# ── 预定义采集位姿（机器人在标定板坐标系下运动）───
# 相机固定(eye-to-hand)，标定板装在末端，机械臂在相机视野内移动
def generate_poses(center_x=200, center_y=0, z=30):
    """生成绕中心点的采集位姿，覆盖平移+旋转"""
    poses = []
    # 不同位置 + 不同旋转
    offsets = [
        (0, 0, 0), (30, 0, 0), (-30, 0, 0),
        (0, 30, 0), (0, -30, 0), (30, 30, 0),
        (-30, -30, 0), (20, -20, 0), (-20, 20, 0),
        (40, 10, 10), (-40, -10, -10), (0, 40, 5),
        (30, -30, -5), (-30, 30, 5), (10, 40, 0),
    ]
    for dx, dy, dz in offsets:
        # 加点小旋转
        for r in [0, 10, -10, 20]:
            poses.append((center_x + dx, center_y + dy, z + dz, r))
            if len(poses) >= N_SAMPLES:
                return poses[:N_SAMPLES]
    return poses[:N_SAMPLES]


def main():
    print("=" * 55)
    print("手眼标定采集 (Eye-to-Hand)")
    print("=" * 55)

    # 连接机械臂
    print("\n[1] 连接 Dobot...")
    arm = DobotArm()
    if not arm.connect("COM3"):
        print("机械臂连接失败！")
        return
    print("已连接")

    # 连接相机
    print("\n[2] 连接海康相机...")
    try:
        from src.perception.hikvision_camera import HikvisionCamera
        cam = HikvisionCamera()
        if not cam.open(0):
            print("相机连接失败！")
            arm.disconnect()
            return
        cam.set_exposure(30000)
        print(f"相机就绪: {cam.resolution}")
    except Exception as e:
        print(f"相机错误: {e}")
        print("使用 OpenCV 通用相机...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("无可用相机！")
            arm.disconnect()
            return
        cam = cap

    # 采集
    print(f"\n[3] 采集 {N_SAMPLES} 组样本...")
    poses = generate_poses()
    samples_R_gripper2base = []
    samples_t_gripper2base = []
    samples_R_target2cam = []
    samples_t_target2cam = []

    for i, (x, y, z, r_deg) in enumerate(poses):
        print(f"\n  [{i+1}/{N_SAMPLES}] 移动到 ({x:.0f}, {y:.0f}, {z:.0f}, r={r_deg}°)")

        # 安全：先升到安全高度
        arm.move_to_pose(Pose(x, y, Z_SAFE, r_deg))
        time.sleep(0.3)
        arm.move_to_pose(Pose(x, y, z, r_deg))
        time.sleep(0.5)  # 等机械臂稳定

        # 采图
        if hasattr(cam, 'grab'):
            frame = cam.grab(timeout_ms=2000)
        else:
            ret, frame = cam.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if frame is None:
            print("    采图失败，跳过")
            continue

        # 检测标定板
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if not found:
            print("    未检测到标定板！请确认:")
            print("    1) 标定板在相机视野内")
            print("    2) 光照充足")
            print("    3) 棋盘格完整可见")
            # 保存调试图
            cv2.imwrite(f"experiments/calib_debug_{i+1}.png", frame[:,:,::-1])
            continue

        # 精化角点
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

        # PnP 求解标定板在相机中的位姿
        ret_pnp, rvec, tvec = cv2.solvePnP(objp, corners, K, DIST)
        if not ret_pnp:
            print("    PnP 求解失败")
            continue
        R_target2cam, _ = cv2.Rodrigues(rvec)

        # 机械臂末端位姿（标定板 = 末端）
        r_rad = np.radians(r_deg)
        R_gripper2base = np.array([
            [np.cos(r_rad), -np.sin(r_rad), 0],
            [np.sin(r_rad),  np.cos(r_rad), 0],
            [0,              0,             1]
        ], dtype=np.float64)
        t_gripper2base = np.array([x, y, z], dtype=np.float64)

        samples_R_gripper2base.append(R_gripper2base)
        samples_t_gripper2base.append(t_gripper2base)
        samples_R_target2cam.append(R_target2cam)
        samples_t_target2cam.append(tvec.ravel())

        print(f"    OK! board_dist={np.linalg.norm(tvec):.0f}mm")

    # 计算手眼矩阵
    n = len(samples_R_gripper2base)
    print(f"\n[4] 计算手眼矩阵 ({n} 组样本)...")

    if n < 5:
        print(f"样本不足 ({n}<5)，无法标定")
        arm.disconnect()
        return

    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        samples_R_gripper2base, samples_t_gripper2base,
        samples_R_target2cam, samples_t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    print(f"\n  R_cam2base:\n{R_cam2base}")
    print(f"\n  t_cam2base (mm): {t_cam2base.ravel()}")

    # 保存
    result = {
        "mode": "eye_to_hand",
        "R": R_cam2base.tolist(),
        "t": t_cam2base.ravel().tolist(),
        "checkerboard": list(CHECKERBOARD),
        "square_size_mm": SQUARE_SIZE_MM,
        "n_samples": n,
        "camera_intrinsics": {
            "fx": float(K[0,0]), "fy": float(K[1,1]),
            "cx": float(K[0,2]), "cy": float(K[1,2]),
        },
    }
    path = "config/hand_eye.yaml"
    os.makedirs("config", exist_ok=True)
    with open(path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\n[5] 标定结果已保存到 {path}")

    # 清理
    if hasattr(cam, 'close'):
        cam.close()
    arm.disconnect()
    print("\n完成!")


if __name__ == "__main__":
    main()
