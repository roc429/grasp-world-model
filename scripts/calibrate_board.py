"""
黑白板手眼标定 (左转15度进入镜头)
机械臂末端固定标定板 → 移动到 N 个位姿 → 检测棋盘格 → 计算手眼矩阵
"""
import sys, os, time, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np, cv2

SQUARE_MM = 20.0           # 方格边长
CHECKERBOARD = (3, 3)      # 内角点 (4x4 方格 → 3x3 内角点)
Z_SAFE = 80.0
ROTATION = -15             # 左转15度

# 采集位姿 (x, y, z, r) — r 固定 -15°
POSES = [
    (160, -60, 25, ROTATION),
    (160, 60, 25, ROTATION),
    (200, -60, 25, ROTATION),
    (200, 60, 25, ROTATION),
    (240, -60, 25, ROTATION),
    (240, 60, 25, ROTATION),
    (180, -80, 25, ROTATION),
    (180, 80, 25, ROTATION),
    (180, 0, 25, ROTATION),
]

# 棋盘格3D点
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1,2) * SQUARE_MM

# 相机内参 (可在 MVS 中获取准确值后更新)
K = np.array([[554.3,0, 2592/2],[0,554.3,1944/2],[0,0,1]], dtype=np.float64)
DIST = np.zeros((5,), np.float64)


def enhance(gray):
    clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(12,12))
    return clahe.apply(gray)


def main():
    print("=" * 50)
    print("黑白板手眼标定 (左转15度)")
    print("=" * 50)

    from src.robot.dobot_arm import DobotArm
    from src.robot.base_arm import Pose
    from src.perception.hikvision_camera import HikvisionCamera

    arm = DobotArm()
    if not arm.connect("COM3"): print("机械臂失败!"); return
    cam = HikvisionCamera()
    if not cam.open(0): print("相机失败!"); arm.disconnect(); return

    samples_R_g2b, samples_t_g2b = [], []
    samples_R_t2c, samples_t_t2c = [], []

    for i, (x, y, z, r) in enumerate(POSES):
        print(f"\n[{i+1}/{len(POSES)}] ({x:.0f},{y:.0f},{z:.0f}) r={r}")
        arm.move_to_pose(Pose(x, y, Z_SAFE, r))
        time.sleep(0.2)
        arm.move_to_pose(Pose(x, y, z, r))
        time.sleep(0.6)

        frame = cam.grab(timeout_ms=2000)
        if frame is None: print("  采图失败"); continue
        frame = np.array(frame)

        gray = enhance(cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY))
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH)
        if not found:
            print("  未检测到棋盘格!")
            cv2.imwrite(f"experiments/board_fail_{i+1:02d}.png", gray)
            continue

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)

        ret, rvec, tvec = cv2.solvePnP(objp, corners, K, DIST)
        if not ret: print("  PnP失败"); continue
        R_t2c, _ = cv2.Rodrigues(rvec)

        r_rad = np.radians(r)
        R_g2b = np.array([[np.cos(r_rad), -np.sin(r_rad), 0],
                          [np.sin(r_rad), np.cos(r_rad), 0],
                          [0,0,1]], dtype=np.float64)
        t_g2b = np.array([x, y, z], dtype=np.float64)

        samples_R_g2b.append(R_g2b)
        samples_t_g2b.append(t_g2b)
        samples_R_t2c.append(R_t2c)
        samples_t_t2c.append(tvec.ravel())
        print(f"  OK! dist={np.linalg.norm(tvec):.0f}mm")

    n = len(samples_R_g2b)
    print(f"\n{n} 组样本, 计算手眼矩阵...")
    if n < 5: print("不足!"); cam.close(); arm.disconnect(); return

    R_c2b, t_c2b = cv2.calibrateHandEye(
        samples_R_g2b, samples_t_g2b, samples_R_t2c, samples_t_t2c,
        method=cv2.CALIB_HAND_EYE_TSAI)

    print(f"R_cam2base:\n{R_c2b}")
    print(f"t_cam2base: {t_c2b.ravel()}")

    # 保存同时生成像素→世界仿射矩阵
    T = np.eye(4); T[:3,:3] = R_c2b; T[:3,3] = t_c2b.ravel()

    json.dump({
        "mode": "eye_to_hand",
        "R": R_c2b.tolist(), "t": t_c2b.ravel().tolist(),
        "T": T.tolist(), "n_samples": n,
    }, open("config/hand_eye.json", "w"), indent=2)
    print("已保存 config/hand_eye.json")

    arm.move_to_pose(Pose(200, 0, Z_SAFE, ROTATION))
    cam.close(); arm.disconnect()
    print("完成!")


if __name__ == "__main__":
    main()
