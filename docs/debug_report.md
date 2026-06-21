# 问题调试报告

> 项目: push-grasp-world-model | 日期: 2026-06-21

---

## 1. Git 分支合并冲突

**问题**: 赵中赐分支 (master) 基于旧代码骨架，合并时删除了 4146 行（滕涛全部仿真代码 + 规划器代码）

**原因**: 赵中赐从初始 commit 创建分支，未基于最新 main

**解决**: 
- 创建 merge/all-progress 分支
- 只 cherry-pick 赵中赐的 5 个新增文件 + 5 个修改文件
- 手动合并 inference.py（保留滕涛的 HeuristicWM + 赵中赐的评分增强）

**教训**: 所有人必须在 main 最新 commit 上创建功能分支

---

## 2. 状态向量格式不统一

**问题**: 队长版使用 `[target, obstacle, goal_x, goal_y, 0, 0]`（米），滕涛版使用 `[target, obstacle, ee, gripper]`（毫米）

**解决**: 统一采用滕涛版本（10 维 mm 单位），与 SimulationEnv._build_state() 格式对齐

**影响范围**: run_task.py, planner, world model 全部适配

---

## 3. Dobot DLL 架构问题

**问题**: DobotDll.so 为 x86_64 Linux 编译，无法在 ARM (Jetson TX2) 上运行

**解决方案**: 优先在 x86 Windows/Linux 上连接 Dobot；TX2 需要交叉编译或用 x86 主机控制

**当前状态**: DobotArm 代码已写，待实机验证

---

## 4. CUDA/GPU 不可用

**问题**: 本地 WSL 无 GPU，`torch.cuda.is_available()` 返回 False

**解决方案**: 
- 本地使用 CPU 推理 + HeuristicWorldModel
- 世界模型训练在 DCU 服务器上进行
- 训练完成后下载 .pt 文件到本地推理

---

## 5. 测试脚本依赖训练模型

**问题**: test_planner.py 硬编码加载 world_model.pt，本地无训练模型时崩溃

**解决方案**: run_task.py 添加自动回退逻辑：有 .pt 用训练模型，没有用 HeuristicWM

**状态**: 已修复

---

## 6. WSL 与 Windows 文件路径

**问题**: WSL 和 PowerShell 之间的路径映射不一致（/c/Users vs C:\Users）

**解决方案**: Git 操作在 PowerShell 执行（网络通），Python 运行在 WSL 执行（有 MuJoCo/Conda）

**当前工作流**:
- PowerShell: git fetch/push/pull
- WSL: conda activate + python scripts/*.py

---

## 7. PyTorch 版本警告

**问题**: `CUDA initialization: The NVIDIA driver on your system is too old (found version 12070)`

**影响**: 无实际影响，使用 CPU 推理，不依赖 GPU

**解决方案**: 忽略警告，或在代码中设置 `CUDA_VISIBLE_DEVICES=""` 禁用 CUDA

---

## 8. 模拟推-抓参数调优

**问题**: 初始推动作参数不合理导致物体飞出桌面

**解决**: 
- 限制推的距离范围 (10-80mm)
- 推高度设为桌面+物体高度+安全余量
- 添加 workspace 边界检查（物体超出范围则规划器评分 -100）

---

*文档版本: v1.0 | 2026-06-21*
