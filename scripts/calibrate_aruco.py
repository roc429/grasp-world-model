"""
手眼标定 — ArUco 方案 (更简单, 不需要棋盘格)
1. 手机显示 aruco_marker.png (桌面)
2. 手机贴在机械臂末端
3. 运行此脚本，自动采集+标定
"""

import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2
import json

# ── 配置 ──────────────────────────────────────────
MARKER_SIZE_MM = 50.0    # 手机上 ArUco 标记的实际边长(mm) — 用尺子量!
Z_SAFE = 80.0            # 安全高度
N_SAMPLES = 15           # 采集组数

# ── 采集位姿 ──────────────────────────────────────
def generate_poses():
    """标定采集位姿 (x, y, z, r_deg)"""
    poses = []
    for dx in [0, 30, -30, 0, 0, 40, -40, 20, -20, 30, -30, 0, 20, -20, 40]:
        for dy in [0, 0, 0, 30, -30, 10, -10, 30, -30, -20, 20, 40, -20, 20, -40]:
            poses.append((200 + dx, dy, 25, 0))
            if len(poses) >= N_SAMPLES:
                return poses
    return poses


def main():
    print("=" * 55)
    print("ArUco 手眼标定 (Eye-to-Hand)")
    print(f"标记边长: {MARKER_SIZE_MM}mm")
    print("=" * 55)

    # ── 连机械臂 ──
    print("\n[1] 连接 Dobot...")
    from src.robot.dobot_arm import DobotArm
    from src.robot.base_arm import Pose
    arm = DobotArm()
    if not arm.connect("COM3"):
        print("失败!"); return
    print("已连接")

    # ── 连相机 ──
    print("[2] 连接相机...")
    from src.perception.hikvision_camera import HikvisionCamera
    cam = HikvisionCamera()
    if not cam.open(0):
        print("失败!"); arm.disconnect(); return
    print(f"就绪: {cam.resolution}")

    # ── ArUco setup ──
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # 相机内参(默认)
    K = np.array([[554.3, 0, cam.resolution[0]/2],
                  [0, 554.3, cam.resolution[1]/2],
                  [0, 0, 1]], dtype=np.float64)
    dist = np.zeros((5,), dtype=np.float64)

    # ── 采集 ──
    print(f"\n[3] 采集 {N_SAMPLES} 组...")
    poses = generate_poses()
    samples_R_g2b = []
    samples_t_g2b = []
    samples_R_m2c = []
    samples_t_m2c = []

    for i, (x, y, z, r_deg) in enumerate(poses):
        print(f"\n  [{i+1}/{N_SAMPLES}] ({x:.0f},{y:.0f},{z:.0f}) r={r_deg}")

        arm.move_to_pose(Pose(x, y, Z_SAFE, r_deg))
        time.sleep(0.2)
        arm.move_to_pose(Pose(x, y, z, r_deg))
        time.sleep(0.5)

        frame = cam.grab(timeout_ms=2000)
        if frame is None:
            print("    采图失败")
            continue

        # 检测 ArUco
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            print("    未检测到标记!")
            cv2.imwrite(f"experiments/aruco_fail_{i+1}.png", frame[:,:,::-1])
            continue

        # 用第一个检测到的标记
        idx = 0
        marker_corners = corners[idx][0]

        # 标记 3D 点 (标记坐标系, Z=0)
        half = MARKER_SIZE_MM / 2.0
        obj_pts = np.array([
            [-half, -half, 0],
            [ half, -half, 0],
            [ half,  half, 0],
            [-half,  half, 0],
        ], dtype=np.float64)

        # PnP: 求解标记在相机中的位姿
        ret, rvec, tvec = cv2.solvePnP(obj_pts, marker_corners, K, dist)
        if not ret:
            print("    PnP 失败"); continue
        R_m2c, _ = cv2.Rodrigues(rvec)

        # 机械臂位姿
        r_rad = np.radians(r_deg)
        R_g2b = np.array([
            [np.cos(r_rad), -np.sin(r_rad), 0],
            [np.sin(r_rad),  np.cos(r_rad), 0],
            [0,              0,             1]
        ], dtype=np.float64)
        t_g2b = np.array([x, y, z], dtype=np.float64)

        samples_R_g2b.append(R_g2b)
        samples_t_g2b.append(t_g2b)
        samples_R_m2c.append(R_m2c)
        samples_t_m2c.append(tvec.ravel())

        # 可视化
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imwrite(f"experiments/aruco_{i+1:02d}.png", frame[:,:,::-1])
        print(f"    OK! dist={np.linalg.norm(tvec):.0f}mm, marker_id={ids[0][0]}")

    # ── 标定计算 ──
    n = len(samples_R_g2b)
    print(f"\n[4] 计算手眼矩阵 ({n} 组)...")
    if n < 5:
        print(f"不足!"); cam.close(); arm.disconnect(); return

    R_c2b, t_c2b = cv2.calibrateHandEye(
        samples_R_g2b, samples_t_g2b,
        samples_R_m2c, samples_t_m2c,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    print(f"\n  R_cam2base:\n{R_c2b}")
    print(f"  t_cam2base (mm): {t_c2b.ravel()}")

    result = {
        "mode": "eye_to_hand",
        "R": R_c2b.tolist(),
        "t": t_c2b.ravel().tolist(),
        "marker_size_mm": MARKER_SIZE_MM,
        "n_samples": n,
    }
    os.makedirs("config", exist_ok=True)
    with open("config/hand_eye_aruco.yaml", "w") as f:
        json.dump(result, f, indent=2)
    print(f"\n[5] 已保存 config/hand_eye_aruco.yaml")

    cam.close()
    arm.disconnect()
    print("完成!")


if __name__ == "__main__":
    main()
