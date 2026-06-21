# 世界模型前瞻规划：推-抓组合任务

> World Model Lookahead Planning: Push-Grasp Combined Task

## 项目简介

当目标物体被障碍物挡住或处于不易抓取位置时，机械臂需要先推开/推近物体，再抓取放置。
本项目基于世界模型的前瞻规划思想，完成单臂推-抓组合任务。

## 技术栈

- **仿真**: MuJoCo 2.3+
- **世界模型**: MLP/GRU (PyTorch) + HeuristicWM
- **规划算法**: CEM / 随机 Shooting
- **机械臂**: Dobot Magician (实机) / 仿真推杆 (SimArm)
- **感知**: 仿真真值 + RGB-D 相机渲染
- **Python**: 3.10

## 项目结构

```
├── config/          # 配置文件 (YAML)
│   ├── scenes/      # 3 种场景布局
├── src/
│   ├── perception/  # 感知模块 (detector + camera + state_encoder)
│   ├── world_model/ # 世界模型 (MLP/GRU/Heuristic + 训练器 + 推理)
│   ├── planner/     # 前瞻规划 (CEM + Shooting)
│   ├── robot/       # 机械臂控制 (SimArm + DobotArm + IK)
│   ├── simulation/  # MuJoCo 仿真环境
│   └── utils/       # 工具 (config + logger + metrics + visualizer)
├── scripts/         # 运行脚本
├── experiments/     # 实验记录
├── models/          # 训练好的模型权重
├── data/            # 数据集
├── docs/            # 文档
└── notebooks/       # Jupyter 笔记本
```

## 快速开始

```bash
# 1. 创建环境
conda create -n push_grasp python=3.10 -y
conda activate push_grasp
pip install numpy scipy matplotlib pyyaml tqdm mujoco torch

# 2. 测试仿真环境
python scripts/test_simulation.py

# 3. 测试感知模块
python scripts/test_perception.py

# 4. 运行完整任务 (使用 HeuristicWM，无需训练模型)
python scripts/run_task.py --layout 2 --trials 10 --planner cem

# 5. 批量评估 (3 布局)
python scripts/evaluate.py

# 6. 训练世界模型 (需要数据)
python scripts/collect_data.py           # 采集数据
python scripts/train_world_model.py      # 训练模型
python scripts/validate_world_model.py   # 验证精度
python scripts/run_ablation.py           # 消融实验
python scripts/generate_charts.py        # 生成图表
```

## 已通过测试

| 测试 | 命令 | 状态 |
|------|------|------|
| 仿真环境 | `python scripts/test_simulation.py` | ✅ |
| 感知模块 | `python scripts/test_perception.py` | ✅ IK ALL PASS / Camera PASS |
| 机械臂连接 | `python scripts/test_arm_connection.py` | ✅ |
| 完整任务 | `python scripts/run_task.py --layout 2 --trials 3` | ✅ 100% |

## 团队分工

- **队长 (贾文鹏)**: 系统集成 + 规划器 + 实验评估
- **滕涛**: 机械臂控制 + 仿真环境 + 相机 + IK
- **赵中赐**: 世界模型训练 + 感知 + 数据采集

## 文档

- `docs/design.md` — 系统设计文档
- `docs/world_model_design.md` — 世界模型设计文档
- `docs/debug_report.md` — 问题调试报告
- `docs/presentation_script.md` — 演示演讲稿
- `docs/tengtao_worklog.md` — 滕涛工作日志

## 评分对照

| 考核内容 | 分值 | 状态 |
|---------|------|------|
| 任务场景与布局设计 | 15 | ✅ |
| 世界模型预测接入 | 20 | ✅ MLP/GRU/Heuristic |
| 前瞻规划算法 | 30 | ✅ CEM + Shooting |
| 推-抓执行效果 | 20 | ✅ 仿真全流程 |
| 文档可视化视频 | 15 | ⚠️ 进行中 |

---

*Repository: https://github.com/roc429/grasp-world-model*
