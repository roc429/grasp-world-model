"""
Dobot Magician 实机控制 — 基于 DobotDllType.py 动态库

依赖:
  - DobotDllType.py (魔术师资料包中)
  - DobotDll.dll (Windows) 或 libDobotDll.so (Linux)

用法:
  from src.robot.dobot_arm import DobotArm

  arm = DobotArm()
  arm.connect(port="COM3")           # Windows: COM3, Linux: /dev/ttyUSB0
  arm.move_to_pose(Pose(200, 0, 50, 0))
  arm.disconnect()
"""

import sys
import time
import logging
from typing import Optional

import numpy as np

from src.robot.base_arm import ArmController, Pose, Action

logger = logging.getLogger(__name__)

# DobotDllType.py 位于魔术师资料包中，不在本仓库内
# 使用时需将其路径加入 sys.path 或放到 src/robot/ 目录下
import os as _os
_dobot_demo_dir = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "dobot_demo")
if _dobot_demo_dir not in sys.path:
    sys.path.insert(0, _dobot_demo_dir)

try:
    import DobotDllType as dType
    DOBOT_DLL_AVAILABLE = True
except ImportError:
    dType = None
    DOBOT_DLL_AVAILABLE = False


class DobotArm(ArmController):
    """
    Dobot Magician 实机控制器。

    基于 Dobot 动态库（DobotDllType.py）封装，实现 ArmController 接口。

    通信方式:
      - USB 串口 (115200 bps)
      - 队列命令模式（QueuedCmd）：命令异步下发，内部排队执行

    安全约束:
      - 运动前自动检查工作空间范围
      - 紧急停止立即清空指令队列
      - 默认速度/加速度限制
    """

    # ── 工作空间安全边界 (mm) ──────────────────────────
    SAFE_X_RANGE = (50, 300)
    SAFE_Y_RANGE = (-150, 150)
    SAFE_Z_RANGE = (-20, 150)
    DEFAULT_VELOCITY = 50       # 速度百分比 (0-100)
    DEFAULT_ACCELERATION = 50   # 加速度百分比 (0-100)

    def __init__(self):
        self.api = None                     # Dobot DLL API 对象
        self._connected = False
        self._port = ""
        self._velocity_ratio = self.DEFAULT_VELOCITY
        self._acceleration_ratio = self.DEFAULT_ACCELERATION

    # ── 连接管理 ────────────────────────────────────────

    def connect(self, port: str = "", baudrate: int = 115200) -> bool:
        """
        连接 Dobot 机械臂。

        Args:
            port: 串口号
                  Windows: "COM3", "COM4" 等
                  Linux: "/dev/ttyUSB0", "/dev/ttyACM0" 等
                  留空则自动搜索
            baudrate: 波特率，Dobot 固定 115200
        """
        if not DOBOT_DLL_AVAILABLE:
            logger.error(
                "DobotDllType.py 未找到！\n"
                "  请从魔术师资料包中复制 DobotDllType.py 到 src/robot/ 目录\n"
                "  资料包路径示例: 魔术师资料/Dobot Demo V2.3-zh/"
                "demo-magician-python-64-master/DobotDllType.py"
            )
            raise ImportError("DobotDllType.py not found. See error message above.")

        self._port = port
        logger.info(f"正在连接 Dobot (port={port or 'auto'}, baudrate={baudrate})...")

        # 加载 DLL
        self.api = dType.load()

        # 连接机械臂
        state = dType.ConnectDobot(self.api, port, baudrate)[0]

        if state != dType.DobotConnect.DobotConnect_NoError:
            logger.error(f"连接失败！错误码: {state}")
            logger.error("请检查: 1) USB 线是否接好 2) 串口号是否正确 3) Dobot 是否通电")
            return False

        self._connected = True
        logger.info("Dobot 连接成功！")

        # 清空指令队列，开始执行
        dType.SetQueuedCmdClear(self.api)
        dType.SetQueuedCmdStartExec(self.api)

        # 设置安全速度/加速度
        dType.SetPTPCommonParams(
            self.api, self._velocity_ratio, self._acceleration_ratio, isQueued=1
        )

        # 读取当前位姿确认通信正常
        pose = self.get_pose()
        logger.info(f"当前位姿: x={pose.x:.1f}, y={pose.y:.1f}, z={pose.z:.1f}, r={pose.r:.1f}")

        return True

    def disconnect(self) -> bool:
        """断开 Dobot 连接"""
        if not self._connected:
            return True

        logger.info("正在断开 Dobot 连接...")
        try:
            dType.SetQueuedCmdStopExec(self.api)
            dType.DisconnectDobot(self.api)
        except Exception as e:
            logger.warning(f"断开时出现警告: {e}")
        finally:
            self._connected = False
            self.api = None

        logger.info("Dobot 已断开")
        return True

    # ── 位姿读取 ────────────────────────────────────────

    def get_pose(self) -> Pose:
        """获取当前末端位姿 (x, y, z, r)"""
        if not self._connected:
            raise RuntimeError("Dobot 未连接，无法读取位姿")

        result = dType.GetPose(self.api)
        # GetPose 返回: (x, y, z, r, joint1Angle, joint2Angle, joint3Angle, joint4Angle)
        return Pose(
            x=float(result[0]),
            y=float(result[1]),
            z=float(result[2]),
            r=float(result[3]),
        )

    def get_joint_angles(self):
        """获取四个关节角度（度）"""
        if not self._connected:
            raise RuntimeError("Dobot 未连接")
        result = dType.GetPose(self.api)
        return {
            "j1": float(result[4]),
            "j2": float(result[5]),
            "j3": float(result[6]),
            "j4": float(result[7]),
        }

    # ── 运动控制 ────────────────────────────────────────

    def move_to_pose(self, target: Pose, mode: str = "PTPMOVLXYZ") -> bool:
        """
        点到点运动 (PTP) — 快速移动到目标位姿。

        Args:
            target: 目标位姿
            mode: PTP 模式
                  "PTPMOVLXYZ"  - 直线运动（推荐，笛卡尔空间直线）
                  "PTPMOVJXYZ"  - 关节运动（更快，但不保证路径）
                  "PTPMOVJANGLE"- 关节角度模式
        """
        if not self._connected:
            logger.warning("Dobot 未连接，跳过运动")
            return False

        self._check_safety(target)

        # 解析 PTP 模式
        mode_map = {
            "PTPMOVLXYZ": dType.PTPMode.PTPMOVLXYZMode,
            "PTPMOVJXYZ": dType.PTPMode.PTPMOVJXYZMode,
            "PTPMOVJANGLE": dType.PTPMode.PTPMOVJANGLEMode,
        }
        ptp_mode = mode_map.get(mode, dType.PTPMode.PTPMOVLXYZMode)

        logger.debug(f"PTP 运动: ({target.x:.1f}, {target.y:.1f}, {target.z:.1f}, {target.r:.1f}) mode={mode}")

        # 下发运动指令（isQueued=1 表示加入队列）
        last_index = dType.SetPTPCmd(
            self.api, ptp_mode,
            target.x, target.y, target.z, target.r,
            isQueued=1
        )[0]

        # 等待运动完成
        self._wait_for_completion(last_index)

        return True

    def move_cp(self, target: Pose, velocity: float = 50.0) -> bool:
        """
        连续路径运动 (CP) — 末端沿直线以恒定速度移动。

        用于推动作等需要精确直线轨迹的场景。

        Args:
            target: 目标位姿
            velocity: 运动速度 mm/s（Dobot 实际使用速度百分比 0-100）
        """
        if not self._connected:
            logger.warning("Dobot 未连接，跳过运动")
            return False

        self._check_safety(target)

        # 将速度映射到 Dobot 速度百分比
        velocity_pct = max(10, min(100, int(velocity)))

        logger.debug(f"CP 运动: ({target.x:.1f}, {target.y:.1f}, {target.z:.1f}) vel={velocity_pct}%")

        dType.SetCPCmd(
            self.api,
            dType.ContinuousPathMode.CPAbsoluteMode,
            target.x, target.y, target.z,
            velocity_pct,
            isQueued=1
        )

        # CP 模式下需要等待执行
        time.sleep(0.1)
        self._wait_for_queue_empty()

        return True

    # ── 末端执行器 ──────────────────────────────────────

    def set_gripper(self, grip: bool) -> bool:
        """
        控制夹爪。

        Args:
            grip: True=闭合，False=松开
        """
        if not self._connected:
            return False

        logger.debug(f"夹爪: {'闭合' if grip else '松开'}")
        dType.SetEndEffectorGripper(self.api, enable=True, grip=grip, isQueued=1)
        time.sleep(0.3)  # 等待夹爪动作完成
        return True

    def set_suction_cup(self, suck: bool) -> bool:
        """
        控制吸盘。

        Args:
            suck: True=吸，False=放
        """
        if not self._connected:
            return False

        logger.debug(f"吸盘: {'吸' if suck else '放'}")
        dType.SetEndEffectorSuctionCup(self.api, enable=True, suck=suck, isQueued=1)
        time.sleep(0.3)
        return True

    def home(self) -> bool:
        """回零 — 机械臂回到零点位置"""
        if not self._connected:
            return False

        logger.info("执行回零...")
        dType.SetHOMECmd(self.api, temp=0, isQueued=1)
        # 回零比较慢，等待久一点
        time.sleep(5)
        self._wait_for_queue_empty()
        logger.info("回零完成")
        return True

    def emergency_stop(self) -> bool:
        """紧急停止 — 立即清空指令队列并停止运动"""
        if not self._connected:
            return False

        logger.warning("!!! 紧急停止 !!!")
        dType.SetQueuedCmdForceStopExec(self.api)
        dType.SetQueuedCmdClear(self.api)
        dType.SetQueuedCmdStartExec(self.api)  # 重新使能
        return True

    # ── 内部工具方法 ────────────────────────────────────

    def _check_safety(self, pose: Pose):
        """检查目标位姿是否在工作空间安全范围内"""
        if not (self.SAFE_X_RANGE[0] <= pose.x <= self.SAFE_X_RANGE[1]):
            raise ValueError(
                f"X={pose.x:.1f} 超出安全范围 {self.SAFE_X_RANGE}"
            )
        if not (self.SAFE_Y_RANGE[0] <= pose.y <= self.SAFE_Y_RANGE[1]):
            raise ValueError(
                f"Y={pose.y:.1f} 超出安全范围 {self.SAFE_Y_RANGE}"
            )
        if not (self.SAFE_Z_RANGE[0] <= pose.z <= self.SAFE_Z_RANGE[1]):
            raise ValueError(
                f"Z={pose.z:.1f} 超出安全范围 {self.SAFE_Z_RANGE}"
            )

    def _wait_for_completion(self, last_index: int, timeout: float = 10.0):
        """等待指定索引的指令执行完成"""
        start = time.time()
        while time.time() - start < timeout:
            dType.dSleep(50)  # Dobot DLL 提供的毫秒级 sleep
            current_idx = dType.GetQueuedCmdCurrentIndex(self.api)[0]
            if current_idx >= last_index:
                return
        logger.warning(f"运动等待超时 ({timeout}s)")

    def _wait_for_queue_empty(self, timeout: float = 10.0):
        """等待指令队列清空"""
        start = time.time()
        while time.time() - start < timeout:
            dType.dSleep(50)
            # 检查剩余指令数
            left = dType.GetQueuedCmdLeftSpace(self.api)[0]
            # left_space 返回队列剩余空间，接近最大值说明队列空了
            # 实际的队列总大小取决于固件，通常为 32
            if left >= 30:
                return
        logger.warning(f"队列等待超时 ({timeout}s)")


# ── 连接测试函数 ────────────────────────────────────────

def test_dobot_connection(port: str = ""):
    """
    快速测试 Dobot 连接。

    用法:
        python -c "from src.robot.dobot_arm import test_dobot_connection; test_dobot_connection('COM3')"
    """
    print("=" * 50)
    print("Dobot Magician 连接测试")
    print("=" * 50)

    if not DOBOT_DLL_AVAILABLE:
        print("\n[ERROR] DobotDllType.py 未找到！")
        print("请将 DobotDllType.py 复制到 src/robot/ 目录下")
        print("参考路径: 魔术师资料/Dobot Demo V2.3-zh/demo-magician-python-64-master/DobotDllType.py")
        return False

    arm = DobotArm()

    try:
        # 1. 连接
        print(f"\n[1/4] 连接 Dobot (port={port or 'auto'})...")
        if not arm.connect(port=port):
            print("连接失败！请检查:")
            print("  - USB 线是否接好")
            print("  - 串口号是否正确 (Windows: COM3, Linux: /dev/ttyUSB0)")
            print("  - Dobot 是否通电（电源灯是否亮）")
            return False
        print("  连接成功!")

        # 2. 读取位姿
        print("\n[2/4] 读取当前位姿...")
        pose = arm.get_pose()
        print(f"  x={pose.x:.1f}, y={pose.y:.1f}, z={pose.z:.1f}, r={pose.r:.1f}")

        # 3. 简单运动测试 (小范围)
        print("\n[3/4] 运动测试 (上移 20mm)...")
        arm.move_to_pose(Pose(pose.x, pose.y, pose.z + 20, pose.r))
        print("  运动完成!")
        time.sleep(0.5)
        arm.move_to_pose(pose)  # 返回原位
        print("  返回原位完成!")

        # 4. 读取关节角度
        print("\n[4/4] 读取关节角度...")
        joints = arm.get_joint_angles()
        print(f"  J1={joints['j1']:.1f}  J2={joints['j2']:.1f}  "
              f"J3={joints['j3']:.1f}  J4={joints['j4']:.1f}")

    except Exception as e:
        print(f"\n[ERROR] 测试失败: {e}")
        return False

    finally:
        arm.disconnect()

    print("\n" + "=" * 50)
    print("Dobot 连接测试完成!")
    print("=" * 50)
    return True
