"""机械臂控制抽象基类"""
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


@dataclass
class Pose:
    """机械臂末端位姿"""
    x: float
    y: float
    z: float
    r: float = 0.0

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, self.r])


@dataclass
class Action:
    """统一动作表示"""
    type: str       # "push", "grasp", "place", "move", "home"
    params: dict    # 动作参数


class ArmController(ABC):
    """机械臂控制抽象基类"""

    @abstractmethod
    def connect(self, port: str = "", baudrate: int = 115200) -> bool:
        pass

    @abstractmethod
    def disconnect(self) -> bool:
        pass

    @abstractmethod
    def get_pose(self) -> Pose:
        pass

    @abstractmethod
    def move_to_pose(self, target: Pose, mode: str = "PTPMOVLXYZ") -> bool:
        pass

    @abstractmethod
    def move_cp(self, target: Pose, velocity: float = 50.0) -> bool:
        pass

    @abstractmethod
    def set_gripper(self, grip: bool) -> bool:
        pass

    @abstractmethod
    def set_suction_cup(self, suck: bool) -> bool:
        pass

    @abstractmethod
    def home(self) -> bool:
        pass

    @abstractmethod
    def emergency_stop(self) -> bool:
        pass

    def execute_action(self, action: Action) -> bool:
        if action.type == "push":
            return self._execute_push(action.params)
        elif action.type == "grasp":
            return self._execute_grasp(action.params)
        elif action.type == "place":
            return self._execute_place(action.params)
        elif action.type == "move":
            return self.move_to_pose(Pose(**action.params))
        elif action.type == "home":
            return self.home()
        else:
            raise ValueError(f"Unknown action type: {action.type}")

    def _execute_push(self, params: dict) -> bool:
        start_x, start_y = params["start_x"], params["start_y"]
        angle = params["direction_angle"]
        dist = params["distance"]
        z_push = params["z_push"]
        end_x = start_x + dist * np.cos(angle)
        end_y = start_y + dist * np.sin(angle)
        self.move_to_pose(Pose(start_x, start_y, z_push + 30, 0))
        self.move_to_pose(Pose(start_x, start_y, z_push, 0))
        self.move_cp(Pose(end_x, end_y, z_push, 0), velocity=50)
        self.move_to_pose(Pose(end_x, end_y, z_push + 30, 0))
        return True

    def _execute_grasp(self, params: dict) -> bool:
        x, y, z = params["x"], params["y"], params["z"]
        r = params.get("r", 0)
        z_safe = params.get("z_safe", z + 30)
        self.set_gripper(False)
        self.move_to_pose(Pose(x, y, z_safe, r))
        self.move_to_pose(Pose(x, y, z, r))
        self.set_gripper(True)
        time.sleep(0.5)
        self.move_to_pose(Pose(x, y, z_safe, r))
        return True

    def _execute_place(self, params: dict) -> bool:
        x, y, z = params["x"], params["y"], params["z"]
        r = params.get("r", 0)
        z_safe = params.get("z_safe", z + 30)
        self.move_to_pose(Pose(x, y, z_safe, r))
        self.move_to_pose(Pose(x, y, z, r))
        self.set_gripper(False)
        time.sleep(0.5)
        self.move_to_pose(Pose(x, y, z_safe, r))
        return True
