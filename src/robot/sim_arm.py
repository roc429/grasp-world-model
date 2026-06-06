"""仿真机械臂控制 (MuJoCo/PyBullet)"""
from src.robot.base_arm import ArmController, Pose


class SimArm(ArmController):
    """仿真机械臂 - 占位实现"""

    def __init__(self, env=None):
        self._env = env
        self._connected = False
        self._current_pose = Pose(200, 0, 100, 0)

    def connect(self, port: str = "", baudrate: int = 115200) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> bool:
        self._connected = False
        return True

    def get_pose(self) -> Pose:
        return self._current_pose

    def move_to_pose(self, target: Pose, mode: str = "PTPMOVLXYZ") -> bool:
        self._current_pose = target
        return True

    def move_cp(self, target: Pose, velocity: float = 50.0) -> bool:
        self._current_pose = target
        return True

    def set_gripper(self, grip: bool) -> bool:
        return True

    def set_suction_cup(self, suck: bool) -> bool:
        return True

    def home(self) -> bool:
        self._current_pose = Pose(200, 0, 100, 0)
        return True

    def emergency_stop(self) -> bool:
        return True
