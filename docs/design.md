# 世界模型前瞻规划：推-抓组合任务 — 设计文档

> 项目: push-grasp-world-model | 日期: 2026-06-21

---

## 1. 系统概述

本项目实现一个基于世界模型前瞻规划的推-抓组合机器人操作系统。当目标物体被障碍物遮挡或处于不可直接抓取的位置时，系统通过世界模型预测推动作的效果，选择最优推，再执行抓取和放置。

核心技术栈: Python 3.10 + MuJoCo 仿真 + PyTorch 世界模型 + CEM/Shooting 规划器

---

## 2. 系统架构

```
┌──────────────────────────────────────────────┐
│                 主控制循环                    │
│                                              │
│  感知模块         世界模型        规划器      │
│  (detector +      (MLP/GRU/      (CEM/       │
│   camera +        Heuristic)     Shooting)   │
│   state_encoder)      │              │       │
│       │               │              │       │
│       ▼               ▼              ▼       │
│  state ────────► predict ──────► plan ───┐   │
│    ▲                                      │   │
│    │         ┌──────────────────┐         │   │
│    └─────────│  仿真环境         │◄────────┘   │
│              │  (SimulationEnv)  │             │
│              │  推/抓/放 物理    │             │
│              └──────────────────┘             │
└──────────────────────────────────────────────┘
```

**模块职责:**

| 模块 | 文件 | 功能 |
|------|------|------|
| 感知 | `src/perception/` | 物体检测、状态编码、相机接口 |
| 世界模型 | `src/world_model/` | 状态预测 (MLP/GRU/Heuristic) |
| 规划器 | `src/planner/` | CEM / Shooting 动作搜索 |
| 机械臂 | `src/robot/` | SimArm (仿真) / DobotArm (实机) |
| 仿真 | `src/simulation/` | MuJoCo 桌面推-抓场景 |
| 工具 | `src/utils/` | 配置、日志、指标、可视化 |

**状态向量 (10维, mm):**
```
[target_x, target_y, target_z, obstacle_x, obstacle_y, obstacle_z,
 ee_x, ee_y, ee_z, gripper_state]
```

**动作向量 (4维):**
```
[start_x_mm, start_y_mm, direction_angle_rad, distance_mm]
```

---

## 3. MuJoCo 仿真环境

仿真引擎基于 MuJoCo，提供桌面推-抓任务的物理模拟。

**场景元素:**
- 桌面: 0.4m × 0.3m
- 目标物体: 红色方块 (15mm³)
- 障碍物: 蓝色方块 (15mm³)
- 推杆: 圆柱形末端执行器
- 放置区: 绿色标记 (半径 30mm)

**3 种场景布局:**

| 布局 | 描述 | 目标位置 | 障碍物 |
|------|------|---------|--------|
| 1 | 无障碍直接抓取 | (200, 0, 15) | 无 |
| 2 | 障碍物遮挡需先推 | (230, 0, 15) | (180, 0, 12.5) |
| 3 | 物体在角落需推近 | (100, 80, 15) | 无 |

**关键 API:**
- `env.reset(layout_id)` → 初始状态
- `env.execute_push(start_xy, angle, distance)` → 推动作
- `env.execute_grasp(obj_xy, obj_z)` → 抓取动作
- `env.execute_place(target_xy, target_z)` → 放置动作

---

## 4. 世界模型设计

世界模型预测给定动作后的环境状态变化，为规划器提供"想象"能力。

### 4.1 MLP 世界模型 (WorldModelMLP)

```
s(10) + a(4) = 14 → Linear(14,256)+ReLU → Linear(256,256)+ReLU → Linear(256,10) → Δs → s'=s+Δs
```

- 残差连接: 预测状态变化量 Δs
- 参数量: ~69K
- RMSE: 4.80mm (200 epochs)

### 4.2 GRU 世界模型 (WorldModelGRU)

```
s(10)+a(4)=14 → Linear(14,256) → GRU(256,256) → Linear(256,10) → Δs
```

- 带隐状态记忆，适合建模时序依赖

### 4.3 启发式世界模型 (HeuristicWorldModel)

- 基于物理常识: 物体沿推方向直线移动
- 摩擦系数经验值: 0.6
- 评分: 物体在工作空间内 + 与障碍物距离
- 用途: 基线对比 + 规划器开发调试

### 4.4 训练

- 数据: 5000+ 条仿真交互数据 (state, action, next_state)
- 硬件: DCU 服务器
- 优化器: Adam, lr=0.001, batch_size=64
- 损失函数: MSE

---

## 5. 规划器设计

### 5.1 Shooting 规划器

随机采样 N 条动作序列 → 世界模型预测轨迹 → 选评分最高者。

```
for i in range(N):
    action = random_sample()
    trajectory = world_model.predict(state, action)
    score = evaluate(trajectory)
    if score > best_score: best_action = action
```

### 5.2 CEM 规划器

迭代优化动作分布: 采样 → 评估 → 选精英 → 更新分布均值/标准差。

```
for iteration in range(K):
    candidates = sample(mean, std, M)
    scores = [evaluate(c) for c in candidates]
    elite = top_k(candidates, scores)
    mean = elite.mean(); std = elite.std()
```

**参数**: horizon=5, candidates=200, CEM iterations=5, elite_ratio=0.2

---

## 6. 实验设计

### 6.1 评价指标

- **成功率**: 目标物体被成功推-抓-放置的比例
- **平均规划步数**: 完成任务的规划次数
- **重规划次数**: 滚动重规划的次数
- **世界模型预测误差**: RMSE (mm)

### 6.2 消融实验

| 变量 | 对比项 |
|------|-------|
| 世界模型 | HeuristicWM vs MLP vs GRU |
| 场景布局 | Layout 1 vs 2 vs 3 |
| 规划器 | Shooting vs CEM |

### 6.3 实验结果

(待批量实验后填入)

| 布局 | Heuristic | MLP | GRU |
|------|-----------|-----|-----|
| 1 | ?% | ?% | ?% |
| 2 | ?% | ?% | ?% |
| 3 | ?% | ?% | ?% |

---

## 7. 已知问题与限制

1. Sim-to-Real Gap: 仿真物理与真实 Dobot 存在差异
2. 世界模型预测精度有限 (RMSE ~5mm)
3. 规划器依赖世界模型质量，模型预测偏差会累积
4. 当前未接入真实相机，感知模块使用仿真真值

---

*文档版本: v1.0 | 2026-06-21*
