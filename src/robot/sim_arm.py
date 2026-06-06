"""仿真机械臂控制 — 驱动 MuJoCo 中的末端执行器"""

from typing import Optional
import numpy as np

from src.robot.base_arm import ArmController, Pose


class SimArm(ArmController):
    """
    仿真机械臂控制器。

    通过 SimulationEnv 控制 MuJoCo 中的 mocap 末端执行器。
    支持 PTP（点到点）和 CP（连续路径）运动模式。
    """

    def __init__(self, env=None):
        """
        Args:
            env: SimulationEnv 实例（或后续传入）
        """
        self._env = env
        self._connected = False
        self._current_pose = Pose(200.0, 0.0, 100.0, 0.0)  # 默认位姿 (mm)
        self._gripper_open = True

    def set_env(self, env):
        """绑定仿真环境（在 env.reset() 之后调用）"""
        self._env = env

    # ── 连接管理 ────────────────────────────────────────

    def connect(self, port: str = "", baudrate: int = 115200) -> bool:
        if self._env is None:
            print("[SimArm] WARNING: env not set, call set_env() first")
            return False
        self._connected = True
        print("[SimArm] Connected to simulation environment")
        return True

    def disconnect(self) -> bool:
        self._connected = False
        print("[SimArm] Disconnected from simulation")
        return True

    # ── 位姿读取 ────────────────────────────────────────

    def get_pose(self) -> Pose:
        if self._connected and self._env is not None:
            state = self._env.get_objects_state()
            ee = state["ee"]  # (3,) mm
            return Pose(x=float(ee[0]), y=float(ee[1]), z=float(ee[2]), r=0.0)
        return self._current_pose

    # ── 运动控制 ────────────────────────────────────────

    def move_to_pose(self, target: Pose, mode: str = "PTPMOVLXYZ") -> bool:
        """
        点到点运动 (PTP)：快速移动到目标位姿。

        Args:
            target: 目标位姿 (x, y, z, r 单位 mm/度)
            mode: 运动模式（仿真中忽略，保留接口兼容）
        """
        if not self._connected or self._env is None:
            self._current_pose = target
            return True

        target_pos = np.array([target.x, target.y, target.z], dtype=np.float64)
        # 使用较快的速度做 PTP 运动
        self._env.move_ee_to(target_pos, velocity=100.0)
        self._current_pose = target
        return True

    def move_cp(self, target: Pose, velocity: float = 50.0) -> bool:
        """
        连续路径运动 (CP)：沿直线以恒定速度移动到目标位姿。

        用于推动作等需要精确直线轨迹的场景。

        Args:
            target: 目标位姿
            velocity: 运动速度 mm/s
        """
        if not self._connected or self._env is None:
            self._current_pose = target
            return True

        target_pos = np.array([target.x, target.y, target.z], dtype=np.float64)
        self._env.move_ee_to(target_pos, velocity=velocity)
        self._current_pose = target
        return True

    # ── 末端执行器 ──────────────────────────────────────

    def set_gripper(self, grip: bool) -> bool:
        """
        控制夹爪。

        Args:
            grip: True=闭合, False=松开
        """
        self._gripper_open = not grip
        if self._env is not None:
            if grip:
                self._env._gripper_open = False
                self._env._gripper_attached = True   # 仿真中闭合夹爪 = 物体附着
            else:
                self._env._gripper_open = True
                self._env._gripper_attached = False  # 松开 = 释放物体
        return True

    def set_suction_cup(self, suck: bool) -> bool:
        """控制吸盘"""
        # 仿真中吸盘与夹爪行为类似
        return self.set_gripper(suck)

    # ── 归零与急停 ──────────────────────────────────────

    def home(self) -> bool:
        """回零：移动到默认安全位置"""
        home_pose = Pose(200.0, 0.0, 100.0, 0.0)
        return self.move_to_pose(home_pose)

    def emergency_stop(self) -> bool:
        """紧急停止"""
        self._connected = False
        print("[SimArm] EMERGENCY STOP")
        return True
