"""
相机接口 — 支持仿真渲染和实机 RGB-D 相机

用法:
  # 仿真相机
  from src.perception.camera import SimCamera
  cam = SimCamera(env)          # env 是 SimulationEnv 实例
  rgb, depth = cam.get_rgbd()   # rgb: (H,W,3), depth: (H,W) mm
  K = cam.get_intrinsics()      # 相机内参

  # 实机相机 (需硬件)
  from src.perception.camera import RealSenseCamera
  cam = RealSenseCamera()
"""

from abc import ABC, abstractmethod
from typing import Tuple, Optional
import numpy as np


# ── 抽象基类 ────────────────────────────────────────────

class CameraInterface(ABC):
    """相机抽象基类"""

    @abstractmethod
    def get_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取 RGB + Depth 图像。

        Returns:
            rgb:   (H, W, 3) uint8
            depth: (H, W) float32, 单位 mm
        """
        pass

    @abstractmethod
    def get_intrinsics(self) -> dict:
        """
        获取相机内参。

        Returns:
            {"fx": float, "fy": float, "cx": float, "cy": float,
             "width": int, "height": int}
        """
        pass


# ── 仿真相机 ────────────────────────────────────────────

class SimCamera(CameraInterface):
    """
    仿真相机 — RGB 用 MuJoCo 渲染，深度用几何真值计算。

    Depth 生成方式: 基于已知物体坐标 + 相机投影模型，
    给出完美真值深度图（比物理渲染更可靠，适合训练/评估）。
    """

    CAMERA_POS = np.array([0.2, 0.0, 0.4])  # 相机世界坐标 (m)

    def __init__(self, env, camera_name: str = "overhead",
                 width: int = 640, height: int = 480):
        self._env = env
        self._camera_name = camera_name
        self._width = width
        self._height = height
        self._renderer = None

    def _ensure_renderer(self):
        if self._renderer is None:
            import mujoco
            self._renderer = mujoco.Renderer(
                self._env.model, self._height, self._width
            )

    def get_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取 RGB + 几何深度图"""
        self._ensure_renderer()
        self._renderer.update_scene(self._env.data, camera=self._camera_name)
        rgb = self._renderer.render().copy()
        depth = self._compute_geo_depth()
        return rgb, depth

    def _compute_geo_depth(self) -> np.ndarray:
        """
        基于几何真值生成深度图 (mm)。

        相机俯视桌面，深度 = 相机到物体/桌面的垂直距离。
        """
        cam_z = self.CAMERA_POS[2]  # 相机高度 (m)
        desk_depth = cam_z * 1000.0  # 桌面深度 (mm)
        depth = np.full((self._height, self._width), desk_depth, dtype=np.float32)

        K = self.get_intrinsics()
        objs = self._env.get_objects_state()

        for name in ["target", "obstacle"]:
            pos_mm = objs.get(name)
            if pos_mm is None:
                continue
            pos_m = pos_mm / 1000.0  # -> m
            dx = pos_m[0] - self.CAMERA_POS[0]
            dy = pos_m[1] - self.CAMERA_POS[1]
            dz = self.CAMERA_POS[2] - pos_m[2]  # 深度

            if dz < 0.01:
                continue

            u = int(K["cx"] + K["fx"] * dx / dz)
            v = int(K["cy"] + K["fy"] * dy / dz)
            r = max(2, int(25 / dz))  # 投影半径 (px)

            u0, u1 = max(0, u - r), min(self._width, u + r + 1)
            v0, v1 = max(0, v - r), min(self._height, v + r + 1)
            if u0 < u1 and v0 < v1:
                depth[v0:v1, u0:u1] = dz * 1000.0

        return depth

    def get_intrinsics(self) -> dict:
        fovy = np.radians(60.0)
        fx = (self._width / 2.0) / np.tan(fovy / 2.0)
        return {
            "fx": float(fx), "fy": float(fx),
            "cx": float(self._width / 2.0), "cy": float(self._height / 2.0),
            "width": self._width, "height": self._height,
        }

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


# ── 实机相机 (占位) ─────────────────────────────────────

class RealSenseCamera(CameraInterface):
    """
    Intel RealSense / 其他 RGB-D 相机接口 (占位)。

    需要 pyrealsense2 或等效驱动。
    """

    def __init__(self):
        self._pipeline = None
        self._intrinsics = None

    def connect(self) -> bool:
        """连接相机"""
        try:
            import pyrealsense2 as rs
            self._pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            profile = self._pipeline.start(config)
            # 获取内参
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()
            intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            self._intrinsics = {
                "fx": intr.fx, "fy": intr.fy,
                "cx": intr.ppx, "cy": intr.ppy,
                "width": intr.width, "height": intr.height,
                "depth_scale": depth_scale,
            }
            return True
        except ImportError:
            print("[RealSenseCamera] pyrealsense2 未安装，使用仿真模式")
            return False
        except Exception as e:
            print(f"[RealSenseCamera] 连接失败: {e}")
            return False

    def get_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        if self._pipeline is None:
            return (
                np.zeros((480, 640, 3), dtype=np.uint8),
                np.zeros((480, 640), dtype=np.float32),
            )
        import pyrealsense2 as rs
        frames = self._pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth = np.asanyarray(depth_frame.get_data(), dtype=np.float32)
        rgb = np.asanyarray(color_frame.get_data())[:, :, ::-1]  # BGR → RGB
        return rgb, depth

    def get_intrinsics(self) -> dict:
        return self._intrinsics or {"fx": 525, "fy": 525, "cx": 320, "cy": 240,
                                     "width": 640, "height": 480}

    def close(self):
        if self._pipeline is not None:
            self._pipeline.stop()
