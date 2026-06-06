# 世界模型前瞻规划：推-抓组合任务

> World Model Lookahead Planning: Push-Grasp Combined Task

## 项目简介

当目标物体被障碍物挡住或处于不易抓取位置时，机械臂需要先推开/推近物体，再抓取放置。
本项目基于世界模型的前瞻规划思想，完成单臂推-抓组合任务。

## 技术栈

- **仿真**: MuJoCo / PyBullet
- **世界模型**: MLP/GRU 状态预测网络 (基础), DreamerV3 RSSM (进阶)
- **规划算法**: CEM / 随机 Shooting / MCTS
- **机械臂**: Dobot Magician (实机) / Franka Panda (仿真)
- **语言**: Python 3.10+

## 项目结构

```
grasp-world-model/
├── config/          # 配置文件 (YAML)
├── src/             # 源代码
│   ├── perception/  # 感知模块
│   ├── world_model/ # 世界模型
│   ├── planner/     # 前瞻规划
│   ├── robot/       # 机械臂控制
│   ├── simulation/  # 仿真环境
│   └── utils/       # 工具
├── scripts/         # 运行脚本
├── experiments/     # 实验记录
├── models/          # 模型权重
├── data/            # 数据集
├── docs/            # 文档
└── tests/           # 测试
```

## 快速开始

```bash
# 1. 创建环境
conda create -n push_grasp python=3.10 -y
conda activate push_grasp

# 2. 安装依赖
pip install -r requirements.txt

# 3. 测试机械臂连接 (仿真模式)
python scripts/test_arm_connection.py

# 4. 数据采集
python scripts/collect_data.py

# 5. 训练世界模型
python scripts/train_world_model.py

# 6. 运行完整任务
python scripts/run_task.py --layout 1 --trials 20
```

## 环境要求

- Python 3.10+
- PyTorch 2.x (DCU 版本需根据设备文档确认)
- MuJoCo 2.3+ 或 PyBullet 3.2+
- OpenCV 4.8+
- (可选) ROS Noetic (实机控制)

## 团队分工

- **贾文鹏**: 系统集成 + 规划器 + 实验评估
- **滕涛**: 机械臂控制 + 仿真 + 相机
- **赵中赐**: 世界模型 + 感知 + 数据

## 文档

详细开发指南请参阅 `docs/开发指南.md`
