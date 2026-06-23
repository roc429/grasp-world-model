"""
手眼标定 & 视觉引导模块

支持两种配置:
  - Eye-to-Hand: 相机固定在外（三脚架/支架），需要 T_base_cam
  - Eye-in-Hand: 相机装在机械臂末端，需要 T_ee_cam

用法:
  # 1. 标定
  calibrator = HandEyeCalibrator(mode="eye_to_hand")
  calibrator.collect_sample(arm, camera)   # 重复 N≥15 次
  calibrator.calibrate()
  calibrator.save("config/hand_eye.yaml")

  # 2. 视觉引导
  servo = VisualServo(arm, camera, "config/hand_eye.yaml")
  world_xyz = servo.detect_and_locate(rgb_frame)  # 像素 → 世界坐标
  arm.move_to_pose(Pose(world_xyz[0], world_xyz[1], z_safe, 0))
"""

import os
import json
import time
import logging
import numpy as np

logger = logging.getLogger(__name__)


class HandEyeCalibrator:
    """手眼标定器"""

    def __init__(self, mode: str = "eye_to_hand"):
        """
        Args:
            mode: "eye_to_hand" (相机固定) | "eye_in_hand" (相机在手上)
        """
        if mode not in ("eye_to_hand", "eye_in_hand"):
            raise ValueError(f"未知模式: {mode}")
        self.mode = mode
        self.samples_R_base2gripper = []  # 机械臂末端旋转矩阵
        self.samples_t_base2gripper = []  # 机械臂末端平移向量
        self.samples_R_target2cam = []    # 标定板在相机中的旋转
        self.samples_t_target2cam = []    # 标定板在相机中的平移

        # 标定结果
        self.R = None  # 旋转矩阵 (3,3)
        self.t = None  # 平移向量 (3,)

        self.checkerboard = (11, 8)  # 内角点数 (cols, rows)
        self.square_size = 20.0      # 方格边长 mm

    # ── 采集 ────────────────────────────────────────

    def collect_sample(self, arm_pose_xyzr, camera_frame):
        """
        采集一组标定数据。

        Args:
            arm_pose_xyzr: 机械臂末端位姿 (x, y, z, r) — r 是绕 Z 轴旋转(度)
            camera_frame: 相机拍摄的标定板图像 (H, W, 3) numpy array

        Returns:
            bool: 是否成功检测到标定板
        """
        import cv2

        gray = cv2.cvtColor(camera_frame, cv2.COLOR_RGB2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, self.checkerboard,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
        )

        if not found:
            logger.warning("未检测到标定板角点")
            return False

        # 精化角点
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

        # 标定板 3D 点（在标定板坐标系中，Z=0）
        objp = np.zeros((self.checkerboard[0] * self.checkerboard[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.checkerboard[0], 0:self.checkerboard[1]].T.reshape(-1, 2)
        objp *= self.square_size

        # 用 PnP 求解标定板在相机中的位姿
        # 需要相机内参 — 先用默认值，后续可替换
        h, w = camera_frame.shape[:2]
        K = np.array([
            [554.3, 0, w / 2],
            [0, 554.3, h / 2],
            [0, 0, 1]
        ], dtype=np.float64)

        dist = np.zeros((5,), dtype=np.float64)
        ret, rvec, tvec = cv2.solvePnP(objp, corners, K, dist)
        if not ret:
            logger.warning("PnP 求解失败")
            return False

        R_target2cam, _ = cv2.Rodrigues(rvec)

        # 机械臂末端位姿 → 旋转+平移
        x, y, z, r_deg = arm_pose_xyzr
        r_rad = np.radians(r_deg)
        R_base2gripper = np.array([
            [np.cos(r_rad), -np.sin(r_rad), 0],
            [np.sin(r_rad),  np.cos(r_rad), 0],
            [0,              0,             1]
        ], dtype=np.float64)
        t_base2gripper = np.array([x, y, z], dtype=np.float64)

        self.samples_R_base2gripper.append(R_base2gripper)
        self.samples_t_base2gripper.append(t_base2gripper)
        self.samples_R_target2cam.append(R_target2cam)
        self.samples_t_target2cam.append(tvec.ravel())

        logger.info(f"采集样本 {len(self.samples_R_base2gripper)}: "
                     f"arm=({x:.0f},{y:.0f},{z:.0f}) "
                     f"board_dist={np.linalg.norm(tvec):.0f}mm")
        return True

    # ── 标定计算 ────────────────────────────────────

    def calibrate(self):
        """执行手眼标定"""
        import cv2

        n = len(self.samples_R_base2gripper)
        if n < 5:
            logger.error(f"样本不足 ({n}<5)，需至少 5 组，推荐 15 组以上")
            return False

        logger.info(f"使用 {n} 组样本进行手眼标定 ({self.mode})")

        if self.mode == "eye_to_hand":
            self.R, self.t = cv2.calibrateHandEye(
                self.samples_R_base2gripper, self.samples_t_base2gripper,
                self.samples_R_target2cam, self.samples_t_target2cam,
                method=cv2.CALIB_HAND_EYE_TSAI
            )
        else:  # eye_in_hand
            self.R, self.t = cv2.calibrateHandEye(
                self.samples_R_base2gripper, self.samples_t_base2gripper,
                self.samples_R_target2cam, self.samples_t_target2cam,
                method=cv2.CALIB_HAND_EYE_TSAI
            )

        # 计算重投影误差
        errors = []
        for i in range(n):
            if self.mode == "eye_to_hand":
                # T_base_cam = T_base_ee @ T_ee_cam
                # 验证: P_base = T_base_cam @ P_cam
                pass
            errors.append(0.0)  # 简化

        logger.info(f"标定完成: R={self.R.tolist() if self.R is not None else None}")
        logger.info(f"          t={self.t.ravel().tolist() if self.t is not None else None}")

        return self.R is not None

    # ── 保存/加载 ────────────────────────────────────

    def save(self, path: str):
        """保存标定结果到 JSON"""
        data = {
            "mode": self.mode,
            "R": self.R.tolist() if self.R is not None else None,
            "t": self.t.ravel().tolist() if self.t is not None else None,
            "checkerboard": list(self.checkerboard),
            "square_size_mm": self.square_size,
        }
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        logger.info(f"标定结果已保存到 {path}")

    def load(self, path: str):
        """加载标定结果"""
        with open(path) as f:
            data = json.load(f)
        self.mode = data["mode"]
        self.R = np.array(data["R"])
        self.t = np.array(data["t"])
        self.checkerboard = tuple(data.get("checkerboard", (11, 8)))
        self.square_size = data.get("square_size_mm", 20.0)
        logger.info(f"标定结果已加载: {path}")
        return True

    @property
    def matrix(self):
        """返回 4×4 齐次变换矩阵"""
        if self.R is None or self.t is None:
            return None
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.t.ravel()
        return T


class VisualServo:
    """视觉引导 — 相机像素坐标 → 机器人世界坐标"""

    def __init__(self, arm, camera, calib_path: str = None):
        """
        Args:
            arm: ArmController 实例
            camera: HikvisionCamera 或 SimCamera 实例
            calib_path: 标定文件路径
        """
        self.arm = arm
        self.camera = camera
        self.calib = HandEyeCalibrator()

        if calib_path and os.path.exists(calib_path):
            self.calib.load(calib_path)

        self._target_pixel = None
        self._target_world = None

    # ── 目标检测 ────────────────────────────────────

    def detect_red_object(self, rgb_frame):
        """
        检测红色物体（目标），返回像素坐标 (cx, cy)。
        可替换为任意检测算法。
        """
        import cv2
        hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)

        # 红色在 HSV 中跨越 0 度
        mask1 = cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 50, 50), (180, 255, 255))
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # 取最大轮廓
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return np.array([cx, cy])

    def detect_blue_object(self, rgb_frame):
        """检测蓝色物体（障碍物）"""
        import cv2
        hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (100, 50, 50), (130, 255, 255))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        return np.array([int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])])

    def detect_aruco(self, rgb_frame, marker_id=None):
        """检测 ArUco 标记，返回 (像素坐标, 位姿)"""
        import cv2
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        params = cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(rgb_frame, aruco_dict, parameters=params)

        if ids is None:
            return None

        results = []
        for i, id_ in enumerate(ids):
            if marker_id is not None and id_[0] != marker_id:
                continue
            c = corners[i][0]
            cx = (c[0][0] + c[2][0]) / 2
            cy = (c[0][1] + c[2][1]) / 2
            results.append((id_[0], np.array([cx, cy]), corners[i]))

        return results if results else None

    # ── 坐标转换 ────────────────────────────────────

    def pixel_to_world(self, pixel_uv, z_mm: float = 0):
        """
        像素坐标 → 机器人世界坐标。

        Args:
            pixel_uv: (u, v) 像素坐标
            z_mm: 物体所在平面高度（桌面=0）

        Returns:
            (x, y) 世界坐标 mm
        """
        if self.calib.R is None:
            logger.error("未加载标定结果，无法转换坐标")
            return None

        intrinsics = self.camera.get_intrinsics()
        fx = intrinsics["fx"]
        fy = intrinsics["fy"]
        cx = intrinsics["cx"]
        cy = intrinsics["cy"]

        u, v = pixel_uv

        # 相机坐标系中的方向向量
        x_cam = (u - cx) / fx
        y_cam = (v - cy) / fy

        # 简化：假设相机垂直向下，物体在桌面平面 Z=z_mm
        # T_base_cam @ [X_cam, Y_cam, Z_cam, 1]^T = [X_base, Y_base, Z_base, 1]^T
        #
        # 使用标定矩阵变换到机器人基坐标系
        cam_pt = np.array([x_cam, y_cam, 1.0])  # 归一化相机坐标
        cam_pt *= (self.calib.t[2] - z_mm)      # 按高度缩放

        world_pt = self.calib.R @ cam_pt + self.calib.t.ravel()

        return world_pt[:2]  # (x_mm, y_mm)

    # ── 端到端 ──────────────────────────────────────

    def locate_and_move(self, z_safe: float = 60, z_grasp: float = 15):
        """
        完整流程: 采图 → 检测目标 → 计算世界坐标 → 移动机械臂。

        Returns:
            bool: 是否成功
        """
        from src.robot.base_arm import Pose

        # 1. 采图
        frame = self.camera.grab()
        if frame is None:
            logger.error("采图失败")
            return False

        # 2. 检测
        pixel = self.detect_red_object(frame)
        if pixel is None:
            logger.warning("未检测到目标物体")
            return False

        logger.info(f"检测到目标 @ pixel ({pixel[0]:.0f}, {pixel[1]:.0f})")

        # 3. 坐标转换
        world_xy = self.pixel_to_world(pixel, z_mm=z_grasp)
        if world_xy is None:
            return False

        logger.info(f"世界坐标: ({world_xy[0]:.1f}, {world_xy[1]:.1f}) mm")

        # 4. 执行抓取
        self.arm.move_to_pose(Pose(float(world_xy[0]), float(world_xy[1]), z_safe, 0))
        self.arm.move_to_pose(Pose(float(world_xy[0]), float(world_xy[1]), z_grasp, 0))
        self.arm.set_gripper(True)
        time.sleep(0.5)
        self.arm.move_to_pose(Pose(float(world_xy[0]), float(world_xy[1]), z_safe, 0))

        return True


# ── 快速偏移补偿 ──────────────────────────────────────

def compute_offset(camera_xy, arm_xy):
    """计算固定偏移量"""
    return np.array(arm_xy) - np.array(camera_xy)


def apply_offset(camera_xy, offset):
    """应用偏移补偿"""
    return np.array(camera_xy) + np.array(offset)
