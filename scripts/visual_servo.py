"""
视觉抓取: 相机检测物体 → 像素→世界坐标 → 机械臂抓取放置
"""

import sys, os, time, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2

Z_SAFE = 60.0
Z_GRASP = 12.0
Z_PLACE = 20.0
PLACE_WORLD = (250, -80)  # 放置区世界坐标

# 加载标定矩阵
with open("config/calib_result.json") as f:
    M = np.array(json.load(f)["matrix"])


def pixel_to_world(cx, cy):
    """像素 → 世界坐标"""
    wx, wy = M @ np.array([cx, cy, 1.0])
    return wx, wy


def enhance(frame):
    """CLAHE 增强 (模拟 MVS 显示效果)"""
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16, 16))
    return clahe.apply(gray)


def detect_red_object(rgb_frame):
    """检测红色物体，返回 (cx, cy) 或 None"""
    hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)
    mask1 = cv2.inRange(hsv, (0, 40, 30), (12, 255, 255))
    mask2 = cv2.inRange(hsv, (168, 40, 30), (180, 255, 255))
    mask = mask1 | mask2
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] < 50:
        return None
    return np.array([M["m10"] / M["m00"], M["m01"] / M["m00"]])


def detect_blue_object(rgb_frame):
    """检测蓝色物体"""
    hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (95, 40, 30), (130, 255, 255))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] < 50:
        return None
    return np.array([M["m10"] / M["m00"], M["m01"] / M["m00"]])


def detect_bright_object(rgb_frame):
    """检测亮色物体 (白纸/反光标记)"""
    gray = enhance(rgb_frame)
    _, mask = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=3)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] < 30:
        return None
    return np.array([M["m10"] / M["m00"], M["m01"] / M["m00"]])


def main():
    from src.robot.dobot_arm import DobotArm
    from src.robot.base_arm import Pose
    from src.perception.hikvision_camera import HikvisionCamera

    print("=" * 55)
    print("视觉抓取")
    print("=" * 55)

    arm = DobotArm()
    if not arm.connect("COM3"):
        print("机械臂失败!"); return
    print("[OK] Dobot 已连接")

    cam = HikvisionCamera()
    if not cam.open(0):
        print("相机失败!"); arm.disconnect(); return
    print(f"[OK] 相机: {cam.resolution}")

    while True:
        print("\n" + "-" * 40)
        print("选项: [r] 红色物体  [b] 蓝色物体  [w] 亮色物体")
        print("      [m] 手动输入像素  [q] 退出")
        choice = input("> ").strip().lower()

        if choice == 'q':
            break

        # 采图
        print("采图中...")
        frame = cam.grab(timeout_ms=2000)
        if frame is None:
            print("采图失败!"); continue

        frame = frame.copy() if hasattr(frame, 'copy') else np.array(frame)

        # 检测
        pixel = None
        if choice == 'r':
            pixel = detect_red_object(frame)
        elif choice == 'b':
            pixel = detect_blue_object(frame)
        elif choice == 'w':
            pixel = detect_bright_object(frame)
        elif choice == 'm':
            try:
                cx, cy = map(float, input("像素 (cx cy): ").split())
                pixel = np.array([cx, cy])
            except:
                continue

        if pixel is None:
            print("未检测到物体!")
            # 保存增强后的图供查看
            vis = cv2.cvtColor(enhance(frame), cv2.COLOR_GRAY2BGR)
            cv2.imwrite("experiments/servo_nofind.png", vis)
            print("已保存 servo_nofind.png (增强图像)")
            continue

        # 坐标转换
        wx, wy = pixel_to_world(pixel[0], pixel[1])
        print(f"检测到物体: 像素({pixel[0]:.0f},{pixel[1]:.0f}) → 世界({wx:.1f},{wy:.1f})mm")

        # 可视化
        vis = frame.copy()
        cv2.circle(vis, (int(pixel[0]), int(pixel[1])), 20, (0, 255, 0), 3)
        cv2.putText(vis, f"({wx:.0f},{wy:.0f})mm", (int(pixel[0]) + 25, int(pixel[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imwrite("experiments/servo_detect.png", vis[:, :, ::-1])

        # 执行抓取
        print(f"\n执行抓取: ({wx:.1f}, {wy:.1f})mm")
        print("[1/4] 移到安全高度...")
        arm.move_to_pose(Pose(wx, wy, Z_SAFE, -30))
        time.sleep(0.3)

        print("[2/4] 下降抓取...")
        arm.move_to_pose(Pose(wx, wy, Z_GRASP, -30))
        arm.set_gripper(True)
        time.sleep(0.5)

        print("[3/4] 抬起...")
        arm.move_to_pose(Pose(wx, wy, Z_SAFE, -30))
        time.sleep(0.3)

        print(f"[4/4] 移到放置区 ({PLACE_WORLD[0]},{PLACE_WORLD[1]})...")
        arm.move_to_pose(Pose(PLACE_WORLD[0], PLACE_WORLD[1], Z_SAFE, -30))
        arm.move_to_pose(Pose(PLACE_WORLD[0], PLACE_WORLD[1], Z_PLACE, -30))
        arm.set_gripper(False)
        time.sleep(0.3)
        arm.move_to_pose(Pose(PLACE_WORLD[0], PLACE_WORLD[1], Z_SAFE, -30))

        print("抓取放置完成!")
        # 可以继续下一个

    arm.move_to_pose(Pose(200, 0, Z_SAFE, -30))
    cam.close()
    arm.disconnect()
    print("完成!")


if __name__ == "__main__":
    main()
