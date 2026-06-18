# 滕涛 — 任务完成状态 & 工作流

> 更新: 2026-06-06 | 项目: push-grasp-world-model

---

## 一、已完成模块

| 文件 | 用途 | 状态 |
|------|------|------|
| `src/robot/base_arm.py` | 机械臂抽象基类 (ArmController, Pose, Action) | ✅ 队长写 |
| `src/robot/sim_arm.py` | 仿真机械臂 — 驱动 MuJoCo | ✅ W2 完成 |
| `src/robot/dobot_arm.py` | 实机机械臂 — 封装 DobotDllType.py | ✅ W2 完成 |
| `src/robot/push.py` | 推动作执行器 (PushExecutor) | ✅ W3 完成 |
| `src/robot/grasp.py` | 抓取/放置执行器 (GraspExecutor) | ✅ W4 完成 |
| `src/robot/ik_solver.py` | 逆运动学求解器 (几何法, FK-IK-FK 误差 0mm) | ✅ W3 完成 |
| `src/simulation/env.py` | MuJoCo 仿真环境 (推/抓/放 + 抓取附着) | ✅ W5 完成 |
| `src/simulation/scene_builder.py` | 场景构建 (YAML → MuJoCo XML) | ✅ W5 完成 |
| `src/perception/camera.py` | 仿真相机 (RGB渲染 + 几何深度图) | ✅ W6 完成 |
| `config/scenes/layout_1.yaml` | 布局1: 无障碍，直接抓取 | ✅ |
| `config/scenes/layout_2.yaml` | 布局2: 障碍物遮挡，需先推 | ✅ |
| `config/scenes/layout_3.yaml` | 布局3: 角落，需先推出来 | ✅ |

---

## 二、测试结果

| 测试 | 命令 | 结果 |
|------|------|------|
| 全流程评估 | `python scripts/evaluate.py` | 3/3 布局通过 |
| 连接测试 | `python scripts/test_arm_connection.py --all-layouts` | 3/3 通过 |
| IK+相机 | `python scripts/test_perception.py` | IK ALL PASS, Camera PASS |
| W1 仿真 | `python scripts/test_simulation.py` | 推+抓+放 全通过 |

---

## 三、架构

```
ArmController (base_arm.py)
├─ SimArm (sim_arm.py)      ← 宿舍台式机 → 仿真 MuJoCo
└─ DobotArm (dobot_arm.py)  ← 实验室     → 实机 Dobot

切换方式: 改 config/default.yaml 中 arm.type
  type: "sim"    → SimArm
  type: "dobot"  → DobotArm
```

---

## 四、宿舍开发环境

```
Conda 环境: push_grasp (Python 3.10)
激活: conda activate push_grasp
核心依赖: numpy, scipy, mujoco, pyyaml, matplotlib, tqdm

测试命令:
  python scripts/test_arm_connection.py --all-layouts
  python scripts/test_perception.py
  python scripts/evaluate.py
```

---

## 五、待完成（阻塞：无硬件）

| 任务 | 阻塞原因 |
|------|---------|
| Dobot 实机连接测试 | 需要 Dobot + USB 线 + DobotDllType.py |
| 相机标定 (实机) | 需要 RGB-D 相机 |
| 实机安全测试 / 运动范围标定 | 需要 Dobot |
| 最终联调 | 等队友模块 + 硬件 |

---

## 六、去实验室之前要准备的

1. **DobotDllType.py** — 从"魔术师资料/Dobot Demo V2.3-zh/demo-magician-python-64-master/"复制到 `src/robot/`
2. **DobotDll.dll** — 同目录
3. **确认串口号** — Windows 通常是 COM3 或 COM4
4. **拉最新代码** — `git pull`

## 七、去实验室之后的流程

```powershell
conda activate push_grasp
git pull

# 1. 先测连接
python scripts/test_arm_connection.py --mode dobot --port COM3

# 2. 单步运动测试
python -c "from src.robot.dobot_arm import DobotArm; from src.robot.base_arm import Pose; a=DobotArm(); a.connect('COM3'); a.move_to_pose(Pose(200,0,50,0)); a.disconnect()"

# 3. 全流程（需要队友的感知+世界模型+规划器就绪）
python scripts/run_task.py --mode dobot --layout 1
```
