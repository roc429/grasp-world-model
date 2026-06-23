"""
无标记标定: 图像差分法定位机械臂末端
不需要标定板、不需要标记、不需要反光贴纸。

原理: 采两张图 (有机械臂 vs 背景), 相减找到机械臂位置,
     在已知的 N 个世界坐标处重复此操作, 解算像素→世界映射。
"""

import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import cv2
import json

N_POINTS = 6
Z_SAFE = 60.0
Z_CALIB = 10.0  # 标定时末端贴近桌面

# 标定位置 (覆盖工作区)
POSES = [
    (160, -80, Z_CALIB, -30),   # 左下
    (160, 80, Z_CALIB, -30),    # 左上
    (240, -80, Z_CALIB, -30),   # 右下
    (240, 80, Z_CALIB, -30),    # 右上
    (200, -80, Z_CALIB, -30),   # 中下
    (200, 80, Z_CALIB, -30),    # 中上
    (200, 0, Z_CALIB, -30),     # 中心
    (120, 0, Z_CALIB, -30),     # 左
    (260, 0, Z_CALIB, -30),     # 右
]


def preprocess(frame):
    """Bayer → RGB → 灰度 → 自动拉伸 (模拟MVS显示效果)"""
    gray_raw = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    # CLAHE 局部自适应均衡 — 比简单拉伸效果好
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16,16))
    return clahe.apply(gray_raw)


def find_arm_by_diff(fg_img, bg_img):
    """前景-背景差分, 找到机械臂末端位置"""
    fg_gray = preprocess(fg_img)
    bg_gray = preprocess(bg_img)

    diff = cv2.absdiff(fg_gray.astype(np.float32), bg_gray.astype(np.float32))

    # 自适应阈值: 用 diff 的均值 + 2*std
    thresh_val = diff.mean() + 2.0 * diff.std()
    thresh_val = max(thresh_val, 5.0)  # 最低阈值
    _, mask = cv2.threshold(diff, thresh_val, 255, cv2.THRESH_BINARY)
    mask = mask.astype(np.uint8)

    # 轻量形态学
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # 找变化区域的质心
    ys, xs = np.where(mask > 0)
    if len(xs) < 20:
        return None, mask

    # 取最下方的变化区域作为末端 (机械臂从上方伸入 → 最低点=末端)
    # 取 y 最大的 10% 像素的质心
    y_thresh = np.percentile(ys, 90)
    tip_ys = ys[ys >= y_thresh]
    tip_xs = xs[ys >= y_thresh]

    if len(tip_xs) < 5:
        # fallback: 用全部变化区域的质心
        cx, cy = np.mean(xs), np.mean(ys)
    else:
        cx, cy = np.mean(tip_xs), np.mean(tip_ys)

    return np.array([cx, cy]), mask


def main():
    print("=" * 50)
    print("差分标定: 图像相减定位末端")
    print("=" * 50)

    from src.robot.dobot_arm import DobotArm
    from src.robot.base_arm import Pose
    from src.perception.hikvision_camera import HikvisionCamera

    arm = DobotArm()
    if not arm.connect("COM3"):
        print("机械臂连接失败!"); return
    print("[OK] Dobot 已连接")

    cam = HikvisionCamera()
    if not cam.open(0):
        print("相机失败!"); arm.disconnect(); return
    print(f"[OK] 相机: {cam.resolution}")

    # ── 先拍背景图 (机械臂移开) ──
    print("\n--- 拍摄背景图 (机械臂移开) ---")
    arm.move_to_pose(Pose(200, 0, Z_SAFE, 0))
    time.sleep(0.3)
    arm.move_to_pose(Pose(100, 150, Z_SAFE, 0))  # 移到角落
    time.sleep(1.0)
    bg_frame = cam.grab(timeout_ms=2000)
    if bg_frame is None:
        print("背景采图失败!"); cam.close(); arm.disconnect(); return
    # 不能用copy因为frame是readonly
    bg = bg_frame.copy() if hasattr(bg_frame, 'copy') else np.array(bg_frame)
    print(f"[OK] 背景: {bg.shape[:2]}")
    cv2.imwrite("experiments/calib_bg.png", bg[:,:,::-1])

    # ── 逐点采集 ──
    print(f"\n--- 采集 {N_POINTS} 个标定点 ---")
    pixel_pts = []
    world_pts = []

    for i, (wx, wy, z, r) in enumerate(POSES):
        print(f"\n[{i+1}/{N_POINTS}] 世界 ({wx:.0f}, {wy:.0f})")
        arm.move_to_pose(Pose(wx, wy, Z_SAFE, r))
        time.sleep(0.2)
        arm.move_to_pose(Pose(wx, wy, z, r))
        time.sleep(0.8)

        fg = cam.grab(timeout_ms=2000)
        if fg is None:
            print("  采图失败"); continue
        fg = fg.copy() if hasattr(fg, 'copy') else np.array(fg)

        pixel, mask = find_arm_by_diff(fg, bg)

        if pixel is not None:
            print(f"  像素: ({pixel[0]:.0f}, {pixel[1]:.0f})")
            world_pts.append([wx, wy])
            pixel_pts.append(pixel)
            # 保存可视化
            vis = fg.copy()
            cv2.circle(vis, (int(pixel[0]), int(pixel[1])), 12, (0,255,0), 3)
            cv2.imwrite(f"experiments/calib_pt_{i+1:02d}.png", vis[:,:,::-1])
            cv2.imwrite(f"experiments/calib_mask_{i+1:02d}.png", mask)
        else:
            print("  未检测到机械臂! 检查:")
            print("    - 机械臂是否在相机视野内?")
            print("    - 桌面/背景是否有变化?")
            vis = fg.copy()
            cv2.imwrite(f"experiments/calib_fail_{i+1:02d}.png", vis[:,:,::-1])

    n = len(world_pts)
    print(f"\n成功: {n}/{N_POINTS} 个点")

    if n < 4:
        print("至少需要 4 个点!"); cam.close(); arm.disconnect(); return

    # ── 解算 ──
    pixel_pts = np.array(pixel_pts, dtype=np.float64)
    world_pts = np.array(world_pts, dtype=np.float64)

    ones = np.ones((n, 1))
    M, _, _, _ = np.linalg.lstsq(
        np.hstack([pixel_pts, ones]), world_pts, rcond=None
    )
    M = M.T

    predicted = np.hstack([pixel_pts, ones]) @ M.T
    errors = np.linalg.norm(predicted - world_pts, axis=1)

    print(f"\n仿射矩阵:\n{M}")
    print(f"误差: mean={errors.mean():.1f}mm, max={errors.max():.1f}mm")

    # 测试: 输入像素得世界坐标
    print("\n" + "=" * 50)
    print("像素→世界 转换器 (输入像素 cx cy, Enter 退出)")
    print(f"矩阵: [{M[0,0]:.3f}, {M[0,1]:.3f}, {M[0,2]:.1f};")
    print(f"       {M[1,0]:.3f}, {M[1,1]:.3f}, {M[1,2]:.1f}]")
    print("=" * 50)

    # 保存
    json.dump({
        "matrix": M.tolist(),
        "n_points": n,
        "error_mean_mm": float(errors.mean()),
        "error_max_mm": float(errors.max()),
    }, open("config/calib_result.json", "w"), indent=2)
    print("已保存 config/calib_result.json")

    # 回到安全位置
    arm.move_to_pose(Pose(200, 0, Z_SAFE, 0))
    cam.close()
    arm.disconnect()
    print("完成!")


if __name__ == "__main__":
    main()
