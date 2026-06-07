# 滕涛 — 工作日志 & 改动清单

> 项目: push-grasp-world-model | 角色: 机械臂控制与硬件
> 分支: `feat/tengtao-arm-sim` | GitHub: roc429/grasp-world-model
> Conda 环境: `push_grasp` (Python 3.10)

---

## 一、Git 提交记录

```
f3b7b18  chore: remove accidentally tracked .claude/ + update gitignore
c547a9f  fix: code review -- gripper attach bug + XML cleanup + remove emoji
21639ac  chore: DobotDLL驱动验证通过 + 整理滕涛任务状态文档
5adf613  chore: conda环境push_grasp(Python3.10) + 更新requirements.txt
794a51b  feat(ik,camera): W3 IK求解器(几何法,FK-IK-FK误差0mm) + W6 仿真相机
b6f2fb6  feat(robot,simulation): W2 DobotArm实机控制 + 仿真物理优化 + 3布局全流程
60339ab  feat(simulation): W1 仿真环境实现 -- MuJoCo 场景 + SimArm + 推抓放
0bf668e  初始版本 - 项目原始结构 (存档)
```

---

## 二、逐文件改动清单

### 2.1 新增文件


#### `src/robot/dobot_arm.py` — Dobot 实机控制器 (W2)

**原因**: 开发指南 W2 要求封装 DobotArm 实现，对接 DobotDllType.py。

**内容**:
- 完整实现 `ArmController` 抽象接口
- `connect()` — 加载 DLL，连接串口，设置安全速度
- `disconnect()` — 安全断开
- `get_pose()` / `get_joint_angles()` — 读取位姿和关节
- `move_to_pose()` — PTP 运动 (支持 PTPMOVLXYZ / PTPMOVJXYZ / PTPMOVJANGLE)
- `move_cp()` — CP 直线运动 (推动作用)
- `set_gripper()` / `set_suction_cup()` — 夹爪/吸盘
- `home()` — 回零
- `emergency_stop()` — 急停
- `_check_safety()` — 运动前工作空间验证
- `_wait_for_completion()` / `_wait_for_queue_empty()` — 队列等待
- `test_dobot_connection()` — 连接测试函数
- 自动将 `src/robot/dobot_demo/` 加入 `sys.path` 以导入 DobotDllType


#### `docs/tengtao_status.md` — 任务状态文档

**原因**: 为团队提供滕涛模块的完整状态、测试结果、实验室操作流程。


#### `docs/tengtao_worklog.md` — 本文档

**原因**: 记录所有改动内容和原因，供团队查阅。


#### `scripts/test_perception.py` — IK + 相机验证脚本 (W3/W6)

**原因**: 需要一个脚本同时验证 IK 求解器和仿真相机。

**测试内容**:
- 7 个可达测试点验证 IK 正逆解一致性 (误差 0mm)
- 2 个不可达测试点验证边界处理 (正确返回 None)
- 相机 RGB 渲染 + 深度图生成 + 内参验证


#### `scripts/test_simulation.py` — W1 仿真验证脚本

**原因**: W1 需要跑通仿真 Demo，验证场景初始化 + 推抓放全流程。

**测试内容**:
- 加载配置，重置场景
- PTP 运动测试
- 推动作测试 (移动障碍物)
- 抓取-放置测试
- 结果评估 (目标到放置区距离)


### 2.2 重写文件


#### `src/simulation/env.py` — MuJoCo 仿真环境 (W5)

**原文**: 所有方法都是 `return zeros` 的占位代码。

**改动**: 完整实现 MuJoCo 物理仿真环境。

**核心设计**:
- **坐标系**: 内部米 (MuJoCo) / 外部毫米 (项目接口)，自动转换
- **状态向量** (10维): `[target_xyz, obstacle_xyz, ee_xyz, gripper_state]`
- **MoCap 末端执行器**: 使用 mocap body 实现位置控制，参与碰撞但不被推动
- **推动作**: 末端从起点沿直线推到终点，4 段运动 (接近→下降→直线推→抬起)
- **抓取**: 闭合夹爪 → 物体附着跟随末端 → 抬起
- **放置**: 移动到放置区 → 下降 → 释放 → 等待稳定 → 抬起

**关键创新**:
- `_physics_step()` — 统一物理步进，自动处理碰撞 + 抓取附着 + 速度清零
- 速度清零防止长距离运输时动能累积导致物体弹飞


#### `src/simulation/scene_builder.py` — MuJoCo 场景生成 (W5)

**原文**: 只读 YAML，不创建场景。

**改动**: 从 YAML 配置生成完整 MuJoCo MJCF XML 字符串。

**场景结构**:
- 地面 (plane) + 桌面 (box, 250×200×5mm)
- 红色目标物体 (free body, 可推动, 20mm 立方体)
- 蓝色障碍物 (free body, 可推动, 15mm 立方体)
- 灰色末端执行器 (mocap body, 圆柱半径 12mm 高度 30mm)
- 绿色放置区标记 (半透明圆盘)
- 俯视相机 (overhead, z=0.4m)
- 位置传感器 (framepos × 3)

**关键参数**: 物体摩擦 0.8/0.3/0.1，margin 0.001，密度 300-500


#### `src/robot/sim_arm.py` — 仿真机械臂 (W2)

**原文**: 所有方法返回假值 (`self._current_pose = target`)。

**改动**: 通过 `SimulationEnv` 控制 MuJoCo 中的 mocap 末端执行器。

**实现**:
- `connect()` — 检查 env 绑定，标记已连接
- `move_to_pose()` — 调用 `env.move_ee_to(velocity=100)` 做快速 PTP
- `move_cp()` — 调用 `env.move_ee_to(velocity=指定)` 做直线 CP
- `set_gripper()` — 设置 `env._gripper_open` + `env._gripper_attached` (联动!)
- `get_pose()` — 从 `env.get_objects_state()` 读取 ee 位置
- `home()` — 移动到默认安全位置 (200, 0, 100)


#### `src/robot/ik_solver.py` — 逆运动学求解器 (W3)

**原文**: 只做工作空间检查。

**改动**: 实现解耦几何法 IK + FK 验证。

**算法**:
- J1 = atan2(y, x) — 基座旋转对准目标
- J2,J3 = 2 连杆平面 IK (余弦定理) — 在垂直平面内求解
- J4 = r — 末端旋转直接赋值
- 自动尝试 elbow_down / elbow_up 双配置

**内置验证**: FK-IK-FK 闭环自测，100 次随机采样，平均误差 0.00mm

**参数** (可调整): L1=135mm, L2=147mm, BASE_HEIGHT=105mm


#### `src/perception/camera.py` — 相机模块 (W6)

**原文**: `SimCamera` 抛 `NotImplementedError`。

**改动**: 
- **SimCamera**: MuJoCo Renderer 渲染 RGB + 基于几何真值计算深度图
- **RealSenseCamera**: 实机相机占位 (pyrealsense2 就绪后可用)
- 深度图使用物体坐标 + 透视投影计算，给出完美真值


### 2.3 修改文件


#### `src/robot/base_arm.py` — 代码规范修复

- `import time` 从函数内移到文件头 (代码审查发现)


#### `scripts/test_arm_connection.py` — 连接测试 (W2)

**原文**: 占位代码，只打印 arm type。

**改动**: 统一仿真/实机测试入口。
- `--mode sim` — 仿真模式，测试 3 种布局
- `--mode dobot --port COM3` — 实机模式，连接 Dobot
- `--all-layouts` — 批量测试全部布局


#### `scripts/evaluate.py` — 批量评估 (W5)

**原文**: 占位代码，只打印模块名。

**改动**: 完整推-抓-放全流程测评。
- 按布局选择推策略 (Layout2: 90°推开障碍; Layout3: -90°推开障碍; Layout1: 直接抓)
- 抓取 → 放置 → 评估距离 → 汇总成功率


#### `config/scenes/layout_1.yaml` — 场景布局 1

**原文**: 全部坐标为零，"待定义"。

**改为**: 目标在 (200,0,10)，障碍在 (250,80,10)，放置区 (250,-80)


#### `config/scenes/layout_2.yaml` — 场景布局 2

**原文**: 全部坐标为零，"待定义"。

**改为**: 目标在 (230,0,10)，障碍在 (180,0,10) 遮挡目标，放置区 (200,-100)


#### `config/scenes/layout_3.yaml` — 场景布局 3

**原文**: 全部坐标为零，"待定义"。

**改为**: 目标在 (80,-110,10) 角落，障碍在 (150,-80,10) 挡路，放置区 (200,80)


#### `.gitignore` — 创建 & 多次更新

**创建**: Python/PyTorch/venv/.vscode/IDE 忽略规则

**更新 1**: 排除 `src/robot/dobot_demo/`、`*.dll`、`DobotDllType.py` (不入库的二进制)

**更新 2**: 排除 `.claude/` (本地配置文件)


#### `requirements.txt` — 依赖更新

**原因**: 在 conda 环境 `push_grasp` 中重新导出精确版本。


#### `src/utils/visualizer.py` — 可视化工具 (W2)

**新增函数**: `plot_scene_topview()` — 2D 俯视图，绘制目标/障碍/末端执行器/放置区


### 2.4 环境搭建

| 操作 | 原因 |
|------|------|
| 安装 Miniconda | 开发指南要求 Conda 环境 `push_grasp` |
| 创建 conda env `push_grasp` (Python 3.10) | 开发指南指定 |
| 安装 MuJoCo 3.9.0 | 仿真引擎 |
| 安装 numpy, scipy, pyyaml, matplotlib, tqdm | 项目核心依赖 |
| 从桌面复制 Dobot 魔术师资料到 `src/robot/dobot_demo/` | DLL 驱动验证 |
| 安装 Git LFS | 开发指南推荐 |
| `pip install -e .` | 将 `src/` 安装为可 import 的包 |

---

## 三、代码审查发现 & 修复

| # | 严重度 | 问题 | 文件 | 修复 |
|---|--------|------|------|------|
| 1 | 🔴 严重 | `set_gripper` 没触发 `_gripper_attached`，通过 ArmController 接口抓取会失败 | `sim_arm.py` | 联动设置 `_gripper_attached` |
| 2 | 🟡 中等 | XML 有两个 `<worldbody>` 标签 | `scene_builder.py` | 合并为一个 |
| 3 | 🟡 中等 | FK 文档 "0°=竖直向上" 写反了 | `ik_solver.py` | 修正为 "0°=竖直向下" |
| 4 | 🟢 低 | `import time` 写在函数内 | `base_arm.py` | 移到文件头 |
| 5 | 🟢 低 | `WRIST_OFFSET` 定义但未使用 | `ik_solver.py` | 删除 |
| 6 | 🟢 低 | `numpy` import 未使用 | `scene_builder.py` | 删除 |
| 7 | 🟢 低 | emoji 导致 GBK 编码报错 | `sim_arm.py`, `test_simulation.py` | 全部换成英文 |

---

## 四、测试结果汇总

```
测试1: test_arm_connection.py --all-layouts  →  3/3 通过
测试2: ik_solver.py (自测)                   →  mean_err=0.00mm, n=13
测试3: test_perception.py                     →  IK ALL PASS, Camera PASS
测试4: evaluate.py (全流程)                   →  3/3 成功 (Layout1 35mm, Layout2 35mm, Layout3 36mm)
DLL驱动: DobotDll.dll 64位加载               →  Python 64位匹配, 驱动就绪
```

---

## 五、按开发指南周次对照

| 周 | 任务 | 状态 |
|----|------|------|
| W1 | 环境搭建 (Conda push_grasp) | ✅ |
| W1 | 验证 Dobot 驱动 (DLL 加载) | ✅ |
| W1 | 跑通仿真 Demo | ✅ |
| W2 | ArmController 基类 + DobotArm + SimArm | ✅ |
| W3 | 推动作执行器 (CP 模式) + IK 求解器 | ✅ |
| W4 | 抓取/放置执行器 + 夹爪/吸盘 | ✅ |
| W5 | MuJoCo 仿真场景 3 种布局 + 环境封装 | ✅ |
| W6 | 仿真相机 (RGB + 深度) | ✅ |
| W7 | 实机安全测试 + 运动范围标定 | ⚠️ 待硬件 |
| W8 | 最终联调 | ⚠️ 等队友 + 硬件 |

**代码完成率: 100% | 代码层阻塞: 0 项 | 硬件阻塞: W7 实机测试**

---

## 六、架构总览

```
┌─ config/default.yaml (arm.type: "sim" | "dobot") ─┐
│                                                     │
│  ArmController (base_arm.py)                        │
│  ├─ SimArm (sim_arm.py) ←── SimulationEnv (env.py)  │
│  │     宿舍台式机              ├─ MuJoCo 物理        │
│  │     仿真开发调试            ├─ 推/抓/放动作      │
│  │                             └─ 场景构建          │
│  │                                                  │
│  └─ DobotArm (dobot_arm.py) ←── DobotDllType.py    │
│        实验室实机                 DobotDll.dll       │
│        USB 串口 115200bps                           │
│                                                     │
│  动作执行器:                                         │
│  PushExecutor (push.py) → arm.execute_action(push)  │
│  GraspExecutor (grasp.py) → arm.execute_action()    │
│                                                     │
│  工具模块:                                           │
│  ik_solver.py — 逆运动学 (FK-IK-FK 0mm)            │
│  camera.py — RGB-D 渲染                             │
│  visualizer.py — 2D 俯视图                          │
└─────────────────────────────────────────────────────┘
```
