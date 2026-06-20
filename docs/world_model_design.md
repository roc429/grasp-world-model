# 世界模型设计文档

> 负责人: 赵中赐  
> 模块: 世界模型 + 感知 + 数据

---

## 1. 状态表示

世界模型的状态向量为固定 10 维浮点向量，与 `SimulationEnv._build_state()` 格式完全对齐：

| 维度 | 符号 | 含义 | 单位 |
|------|------|------|------|
| 0-2 | target_xyz | 目标物体位置 (x, y, z) | mm |
| 3-5 | obstacle_xyz | 障碍物位置 (x, y, z) | mm |
| 6-8 | ee_xyz | 末端执行器位置 (x, y, z) | mm |
| 9 | gripper_state | 夹爪状态 (1.0=开, 0.0=闭) | — |

动作向量为 4 维：

| 维度 | 含义 | 范围 |
|------|------|------|
| 0 | push_start_x | 推起点 x (mm) |
| 1 | push_start_y | 推起点 y (mm) |
| 2 | push_angle | 推方向角 (rad) |
| 3 | push_distance | 推距离 (mm) |

---

## 2. 模型结构

### 2.1 MLP 世界模型 (WorldModelMLP)

```
s(10) + a(4) = 14
       ↓
  Linear(14, 256) + ReLU
       ↓
  Linear(256, 256) + ReLU
       ↓
  Linear(256, 10) → Δs
       ↓
  s' = s + Δs
```

- **残差连接**: 预测状态变化量 Δs，而非绝对状态，利于梯度传播
- **参数量**: ~69K
- **适用**: 单步动力学预测，不依赖历史

### 2.2 GRU 世界模型 (WorldModelGRU)

```
s(10) + a(4) = 14
       ↓
  Linear(14, 256) → input_proj
       ↓
  GRU(256, 256, batch_first=True) → 隐状态 h
       ↓
  Linear(256, 10) → Δs
       ↓
  s' = s + Δs
```

- **隐状态**: 256 维，在多步 rollout 中跨时间步传递
- **参数量**: ~270K
- **适用**: 多步前瞻规划，能在 rollout 中利用历史信息

### 2.3 选择建议
- MLP: 训练快，适合数据少 / 消融实验
- GRU: 预测更准，适合最终部署

---

## 3. 训练方法

### 3.1 数据采集

在 MuJoCo 仿真中随机推动作，收集 (state, action, next_state) 转移对：

```
for episode in 1..400:
    layout = random(1, 2, 3)
    env.reset(layout)
    for step in 1..15:
        action = random_push_action()
        next_state, reward, done = env.step(action)
        save(state, action, next_state)
        state = next_state
```

目标: ~5000 条转移，覆盖 3 种场景布局。

### 3.2 损失函数

均方误差 (MSE):

```
L = (1/N) * Σ ||pred_next_state - true_next_state||²
```

### 3.3 超参数

| 参数 | 值 | 说明 |
|------|-----|------|
| learning_rate | 0.001 | AdamW 优化器 |
| batch_size | 64 | — |
| epochs | 100 | MLP 收敛较快可减半 |
| weight_decay | 1e-4 | L2 正则化 |
| hidden_dim | 256 | 隐藏层维度 |
| train/val/test | 80% / 10% / 10% | 固定随机种子 42 |

### 3.4 训练平台

- **DCU 服务器**: `172.19.128.219:6080`
- **框架**: PyTorch (DCU 适配版)
- **设备**: CUDA (DCU accelerator)

---

## 4. 评分函数

用于规划器在前瞻时评估每个预测状态的优劣：

```
score = graspable_weight × gripper_open
      − distance_weight × ||ee_xy − target_xy||
      + collision_penalty × is_collision
      − z_out_of_range_penalty
```

| 参数 | 默认值 | 含义 |
|------|--------|------|
| graspable_weight | 1.0 | 夹爪已打开则加分 |
| distance_weight | 0.01 | 末端越近目标越好 (mm 量级用小权重) |
| collision_penalty | -10.0 | 障碍物距目标 < 30mm 且 z>1mm 时触发 |
| z_out_of_range | -5.0 | 目标 z < -50 或 > 200 时 |

---

## 5. 推理接口

```python
from src.world_model.inference import WorldModel

wm = WorldModel("models/world_model/world_model.pt", config)

# 单步预测
next_state, score, hidden = wm.predict(state, action)

# 轨迹 rollout (供规划器使用)
trajectory, scores = wm.predict_trajectory(state, action_sequence)
# trajectory: (H+1, 10), scores: (H,)
```

---

## 6. 评估指标

- **MSE**: 世界模型在测试集上的均方预测误差
- **成功率**: `run_task.py` 中完成任务的比例
- **平均步数**: 完成任务所需的平均推动作步数

---

## 7. 文件索引

| 文件 | 用途 |
|------|------|
| `src/world_model/model.py` | MLP / GRU 模型定义 |
| `src/world_model/dataset.py` | PushGraspDataset |
| `src/world_model/trainer.py` | 训练循环 + 评估 |
| `src/world_model/inference.py` | 推理封装 + 评分 |
| `src/perception/state_encoder.py` | 状态编码 |
| `scripts/collect_data.py` | 数据采集 |
| `scripts/train_world_model.py` | 命令行训练入口 |
| `notebooks/train_world_model.ipynb` | Notebook 全流程 |
| `scripts/run_ablation.py` | 消融实验 |
| `scripts/generate_charts.py` | 图表生成 |
