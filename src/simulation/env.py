"""MuJoCo 仿真环境 — 推-抓任务的物理模拟"""

import numpy as np
from typing import Tuple, Dict, Any, Optional
import mujoco

from src.simulation.scene_builder import load_scene_config, create_default_scene


class SimulationEnv:
    """
    MuJoCo 仿真环境统一接口。

    坐标系约定：
      - 内部使用米（MuJoCo 标准）
      - 对外接口使用毫米（与项目其他模块一致）
      - X: 前后（前方为正）, Y: 左右（左方为正）, Z: 上下（上方为正）

    状态向量格式 (10维):
      [target_x, target_y, target_z, obstacle_x, obstacle_y, obstacle_z,
       ee_x, ee_y, ee_z, gripper_state]
    """

    def __init__(self, config: dict):
        self.config = config
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self._objects: Dict[str, np.ndarray] = {}
        self._ee_pos = np.zeros(3)           # 末端执行器位置 (m)
        self._gripper_open = True
        self._gripper_attached = False       # 是否已抓取目标物体
        self._target_body_id = -1
        self._obstacle_body_id = -1
        self._ee_mocap_id = -1
        self._layout_id = 1
        self._renderer: Optional[mujoco.Renderer] = None

    # ── 毫米/米转换 ────────────────────────────────────────

    @staticmethod
    def _m2mm(val):
        """米 → 毫米"""
        return val * 1000.0

    @staticmethod
    def _mm2m(val):
        """毫米 → 米"""
        return val / 1000.0

    # ── 场景生命周期 ───────────────────────────────────────

    def reset(self, layout_id: int = 1) -> np.ndarray:
        """
        重置场景并返回初始状态向量 (10,) 单位 mm。

        Args:
            layout_id: 场景布局编号 (1/2/3)
        """
        self._layout_id = layout_id
        scene_config = load_scene_config(layout_id)
        self.model, self.data = create_default_scene(scene_config)

        # 缓存 body id 和 mocap id
        self._target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "target_body")
        self._obstacle_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "obstacle_body")
        ee_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "ee_mocap")
        # mocap_pos 的索引是 mocap body 的序号，不是 body id
        self._ee_mocap_id = self.model.body_mocapid[ee_body_id]

        # 缓存 free joint 的 qpos 地址（用于抓取时移动物体）
        target_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "target_joint")
        self._target_qpos_adr = self.model.jnt_qposadr[target_joint_id]

        self._gripper_open = True
        self._gripper_attached = False

        # 初始化物理（让物体落在桌面上）
        mujoco.mj_forward(self.model, self.data)

        return self._build_state()

    def close(self):
        """关闭仿真环境，释放渲染器"""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self.model = None
        self.data = None

    # ── 状态读取 ────────────────────────────────────────────

    def _build_state(self) -> np.ndarray:
        """
        构建当前状态向量 (10,) 单位 mm。

        格式: [target_x, target_y, target_z, obstacle_x, obstacle_y, obstacle_z,
               ee_x, ee_y, ee_z, gripper_state]
        """
        # 从传感器读取物体位置
        target_pos = self.data.sensor("target_pos").data.copy()      # (3,) meters
        obstacle_pos = self.data.sensor("obstacle_pos").data.copy()  # (3,) meters
        ee_pos = self.data.sensor("ee_pos").data.copy()              # (3,) meters

        state = np.zeros(10, dtype=np.float32)
        state[0:3] = self._m2mm(target_pos)
        state[3:6] = self._m2mm(obstacle_pos)
        state[6:9] = self._m2mm(ee_pos)
        state[9] = 1.0 if self._gripper_open else 0.0
        return state

    def get_objects_state(self) -> Dict[str, np.ndarray]:
        """
        获取所有物体的真实位置（mm）。

        Returns:
            {"target": (3,) mm, "obstacle": (3,) mm, "ee": (3,) mm}
        """
        state = self._build_state()
        return {
            "target": state[0:3],
            "obstacle": state[3:6],
            "ee": state[6:9],
        }

    # ── 渲染 ────────────────────────────────────────────────

    def get_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取仿真 RGB-D 图像（俯视相机视角）。

        Returns:
            rgb: (480, 640, 3) uint8
            depth: (480, 640) float32 in mm
        """
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 640)

        # 设置俯视相机
        self._renderer.update_scene(
            self.data,
            camera="fixed" if hasattr(self, '_fixed_camera') else None,
            scene_option=mujoco.MjvOption()
        )

        # 使用默认相机或手动设置
        try:
            self._renderer.update_scene(self.data, camera=0)
        except Exception:
            # 某些 MuJoCo 版本不支持 camera=0，回退
            self._renderer.update_scene(self.data)

        rgb = self._renderer.render().copy()

        # Depth: 从 depth renderer 获取
        self._renderer.enable_depth_rendering()
        depth_m = self._renderer.render().copy()
        self._renderer.disable_depth_rendering()

        # 深度转毫米
        depth_mm = (depth_m * 1000.0).astype(np.float32)

        return rgb, depth_mm

    # ── 执行器控制（供 SimArm 调用）─────────────────────────

    def set_ee_position(self, pos_mm: np.ndarray):
        """
        设置末端执行器的目标位置（mm），一步到位。

        Args:
            pos_mm: (3,) [x, y, z] in mm
        """
        pos_m = pos_mm.astype(np.float64) / 1000.0
        # 设置 mocap body 位置
        self.data.mocap_pos[self._ee_mocap_id] = pos_m
        mujoco.mj_forward(self.model, self.data)

    def move_ee_to(self, target_mm: np.ndarray, velocity: float = 50.0,
                   steps_per_sec: int = 500):
        """
        将末端执行器平滑移动到目标位置（直线轨迹）。

        Args:
            target_mm: (3,) 目标位置 [x, y, z] in mm
            velocity: 移动速度 mm/s
            steps_per_sec: 每秒物理步数
        """
        start_mm = self._build_state()[6:9]  # 当前位置
        target_mm = np.asarray(target_mm, dtype=np.float64)

        dist_mm = np.linalg.norm(target_mm - start_mm)
        if dist_mm < 0.5:   # 距离太短，直接设置
            self.set_ee_position(target_mm)
            return

        duration = dist_mm / velocity                     # 秒
        num_steps = max(10, int(duration * steps_per_sec))

        for i in range(num_steps + 1):
            alpha = i / num_steps
            pos_mm = start_mm + alpha * (target_mm - start_mm)
            self.set_ee_position(pos_mm)
            # 步进物理（检测碰撞、推动物体）
            mujoco.mj_step(self.model, self.data)

        # 确保精确到达
        self.set_ee_position(target_mm)

    def execute_push(self, start_xy_mm: np.ndarray, direction_angle: float,
                     distance_mm: float, z_push_mm: float = 20.0):
        """
        执行推动作：末端从起点沿直线推到终点。

        Args:
            start_xy_mm: (2,) 起点 [x, y] in mm
            direction_angle: 推的方向角（弧度，0=+X方向）
            distance_mm: 推的距离 mm
            z_push_mm: 推的高度 mm（桌面高度 + 物体半高）

        Returns:
            success: bool
        """
        start_x, start_y = float(start_xy_mm[0]), float(start_xy_mm[1])
        angle = float(direction_angle)
        dist = float(distance_mm)

        end_x = start_x + dist * np.cos(angle)
        end_y = start_y + dist * np.sin(angle)

        z_approach = z_push_mm + 30.0   # 接近高度（物体上方）
        # z_push_mm 已经是桌面 + 物体半高的高度

        # 1. 移动到起点上方
        self.move_ee_to(np.array([start_x, start_y, z_approach]), velocity=80)

        # 2. 下降到推的高度
        self.move_ee_to(np.array([start_x, start_y, z_push_mm]), velocity=40)

        # 3. 直线推动
        self.move_ee_to(np.array([end_x, end_y, z_push_mm]), velocity=50)

        # 4. 抬起
        self.move_ee_to(np.array([end_x, end_y, z_approach]), velocity=80)

        return True

    def execute_grasp(self, obj_xy_mm: np.ndarray, obj_z_mm: float,
                      z_safe_mm: Optional[float] = None):
        """
        执行抓取动作。

        Args:
            obj_xy_mm: (2,) 物体 [x, y] in mm
            obj_z_mm: 物体顶面高度 mm
            z_safe_mm: 安全高度 mm（不指定则自动计算）
        """
        x, y = float(obj_xy_mm[0]), float(obj_xy_mm[1])
        z = float(obj_z_mm)

        if z_safe_mm is None:
            z_safe_mm = z + 30.0

        # 1. 移动到物体上方安全高度
        self.move_ee_to(np.array([x, y, z_safe_mm]), velocity=80)

        # 2. 下降到抓取高度
        self.move_ee_to(np.array([x, y, z - 5.0]), velocity=40)

        # 3. 闭合夹爪（仿真中通过记录实现）
        self._gripper_open = False
        self._gripper_attached = True

        # 短暂停留（让物理稳定），同时让目标物体跟随末端执行器
        for _ in range(250):
            if self._gripper_attached:
                # 将目标物体位置设置为末端执行器位置 (模拟抓取)
                ee_pos = self.data.mocap_pos[self._ee_mocap_id].copy()
                adr = self._target_qpos_adr
                self.data.qpos[adr:adr + 3] = ee_pos
            mujoco.mj_step(self.model, self.data)

        # 4. 抬起
        self.move_ee_to(np.array([x, y, z_safe_mm]), velocity=80)

    def execute_place(self, target_xy_mm: np.ndarray, target_z_mm: float,
                      z_safe_mm: Optional[float] = None):
        """
        执行放置动作。

        Args:
            target_xy_mm: (2,) 放置位置 [x, y] in mm
            target_z_mm: 放置高度 mm
            z_safe_mm: 安全高度
        """
        x, y = float(target_xy_mm[0]), float(target_xy_mm[1])
        z = float(target_z_mm)

        if z_safe_mm is None:
            z_safe_mm = z + 30.0

        # 1. 移动到放置区上方
        self.move_ee_to(np.array([x, y, z_safe_mm]), velocity=80)

        # 2. 下降
        self.move_ee_to(np.array([x, y, z + 5.0]), velocity=40)

        # 3. 松开夹爪
        self._gripper_open = True
        self._gripper_attached = False

        # 让物理稳定
        for _ in range(100):
            mujoco.mj_step(self.model, self.data)

        # 4. 抬起
        self.move_ee_to(np.array([x, y, z_safe_mm]), velocity=80)

    # ── Gym-like 接口 ───────────────────────────────────────

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """
        执行动作并返回 (next_state, reward, done, info)。

        状态和动作单位均为 mm。

        Args:
            action: (4,) [push_x, push_y, push_dir, push_dist]
                    或 [-1, ...] 表示抓取/放置等特殊动作

        Returns:
            (next_state, reward, done, info)
        """
        # 简单实现：对于推动作，执行 push
        if action[0] >= 0:
            self.execute_push(
                start_xy_mm=action[0:2],
                direction_angle=float(action[2]),
                distance_mm=float(action[3]),
                z_push_mm=20.0,
            )

        state = self._build_state()
        reward = self._compute_reward(state)
        done = self._check_done(state)

        return state, reward, done, {}

    def _compute_reward(self, state: np.ndarray) -> float:
        """简单奖励：目标越靠近放置区奖励越高"""
        target = state[0:2]
        placement_xy = self._get_placement_xy_mm()
        dist = np.linalg.norm(target - placement_xy)
        # 距离越近奖励越高
        return float(np.exp(-dist / 50.0))

    def _check_done(self, state: np.ndarray) -> bool:
        """检查任务是否完成"""
        target = state[0:2]
        placement_xy = self._get_placement_xy_mm()
        dist = np.linalg.norm(target - placement_xy)
        return dist < 15.0  # 15mm 以内算到达

    def _get_placement_xy_mm(self) -> np.ndarray:
        """获取放置区位置 (mm)"""
        scene_config = load_scene_config(self._layout_id)
        p = scene_config["placement_zone"]["position"]
        return np.array([p[0], p[1]], dtype=np.float32)
