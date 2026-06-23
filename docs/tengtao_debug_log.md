# 滕涛 — 实机调试日志

> 日期: 2026-06-23 | 机器: 工控机 (KN1GHT)  
> 设备: Dobot Magician + 海康 MV-CE050-30UC + SenseLock Elite4

---

## 一、硬件环境

| 设备 | 接口 | 状态 |
|------|------|------|
| Dobot Magician | USB→COM3 (CP210x) | ✅ 通讯正常 |
| 海康相机 | USB3 (VID_2BDF) | ✅ MVS 可显示 |
| SenseLock Elite4 | USB (VID_3386) | ❌ 硬件不识别 |
| 气泵 | Dobot 末端 | ✅ 气阀控制 |

---

## 二、已解决问题

### 2.1 Dobot 实机控制

| 问题 | 症状 | 根因 | 修复 |
|------|------|------|------|
| DLL 缺失 | `DobotDllType.py not found` | 未从魔术师资料包复制 | 复制 `DobotDllType.py` + `DobotDll.dll` + `Qt5*.dll` + `msvc*.dll` 到项目根目录 |
| API 参数名不匹配 | `SetEndEffectorGripper() got unexpected keyword argument 'enable'` | DLL 函数签名是位置参数，代码用了关键字参数 | 改为位置参数: `dType.SetEndEffectorGripper(self.api, True, grip, 1)` |
| 机械臂不动（红灯） | 脚本报成功但物理不动 | Key 开关未开，电机被硬件锁死 | 拧 Key 开关到 ON，灯变绿即可 |
| 运动队列阻塞 | `运动等待超时` 60s 耗光 | `SetEndEffectorGripper` 的 `isQueued=1` 进入队列，夹爪不存在时命令无法完成，阻塞整个队列 | 改为 `isQueued=0`（直接执行，不排队） |
| 缺少初始化参数 | 电机参数未初始化 | `connect()` 缺少 `SetHOMEParams` + `SetPTPJointParams` | 在 `connect()` 中添加完整初始化序列 |
| 队列执行顺序 | `SetQueuedCmdStartExec` 在空队列时调用 | 官方 Demo 是排队所有指令后再启动 | 在 `_wait_for_completion()` 中每次运动前动态调用 |
| 安全范围过窄 | `X=-11 超出安全范围` | Dobot 原点 X 可能为负值 | 扩展 `SAFE_X_RANGE` 到 `(-50, 300)` |
| `GetQueuedCmdLeftSpace` 不存在 | `AttributeError` | 此版本 DLL 无此函数 | 改为基于 `GetQueuedCmdCurrentIndex` 的队列空闲检测 |
| PyTorch 缺失 | `ModuleNotFoundError: torch` | conda env 未装 | `pip install torch` (清华镜像) |
| 终端输出乱码 | 所有中文显示为 `???` | Windows CMD 默认 GBK 编码 | `etc/conda/activate.d/encoding_fix.bat`: `chcp 65001` + `PYTHONIOENCODING=utf-8` |

### 2.2 run_task.py 实机支持

| 问题 | 修复 |
|------|------|
| `run_task.py` 硬编码 `SimArm` | 新增 `--mode dobot` 参数 + `run_trial_dobot()` 函数 |
| 实机无物体检测 | 硬编码物体坐标常量 `DOBOT_SCENES` |
| ⚠ emoji 编码错误 | 改为 `[WARNING]` 文本 |

**验证结果**: Layout 1/2/3 全部通过 Dobot 实机测试，Layout 2 推-抓-放成功率 100%。

### 2.3 海康相机 SDK

| 问题 | 症状 | 修复 |
|------|------|------|
| DLL 加载 | `Could not find MvCameraControl.dll` | `os.add_dll_directory(MVS_BIN)` 在导入前设置 |
| 设备占用 | `OpenDevice 0x80000203` | MVS 客户端和 SDK 不能同时连接，关掉 MVS |
| 图像极暗 | 原始 range=[3,105], mean=26 | **未解决** — MVS 显示亮是软件拉伸效果，原始数据本身暗 |
| 像素格式 | `0x2180014` 不匹配标准 RGB8 | 设 `PixelType_Gvsp_BayerRG8` 后用 `cv2.COLOR_BayerRG2RGB` 转换 |
| 曝光控制 | `SetFloatValue('ExposureTime')` 不生效 | **未解决** — 相机不接受 SDK 曝光设置 |

**验证结果**: 相机 SDK 可以采图（2592×1944，RGB），但图像偏暗需 CLAHE 增强。

### 2.4 相机标定

| 方案 | 方法 | 结果 |
|------|------|------|
| 棋盘格检测 | `findChessboardCorners` 多种尺寸 | ❌ 全部失败 — 图像太暗+板子尺寸不匹配 |
| ArUco 标记 | 手机显示 marker | ❌ 未检测到 |
| 差分法（无标记） | 背景-前景差分定位末端 | ✅ 4点标定成功: mean=0.3mm |
| 差分法（扩展） | 9点覆盖全工作区 | ❌ mean=57mm — 边缘点检测不准 |

**最佳结果**: `config/calib_result.json`（4点，0.3mm 精度，但范围有限）

### 2.5 加密狗

| 问题 | 症状 | 根因 |
|------|------|------|
| SenseLock Elite4 不识别 | USB 枚举无 VID_3386 | 狗灯亮（供电通）但数据不通 — 疑似硬件损坏 |
| 云软锁心跳失败 | `send_heartbeat failed[10057]` | 授权服务器网络不通 |
| slusb 驱动 | 未安装 | 已通过 `sc create` 安装并启动 |

**结论**: 放弃加密狗方案，用自主代码替代 DobotVisionStudio 全部功能。

---

## 三、遗留问题

1. **相机曝光不可控** — SDK `SetFloatValue('ExposureTime')` 不生效，需用 MVS 手动调好后关闭 MVS 再采图
2. **标定覆盖范围不足** — 4点标定精度高但范围有限，需用黑白板方案补充
3. **气泵抓取未验证** — `set_suction_cup()` 调用了但未确认物理上是否吸到

---

## 四、关键文件变更

| 文件 | 变更 |
|------|------|
| `src/robot/dobot_arm.py` | 修复 API 调用、安全范围、初始化序列、`_wait_for_completion`/`_wait_for_queue_empty` |
| `scripts/run_task.py` | 新增 `--mode dobot` + `run_trial_dobot()` |
| `config/default.yaml` | `arm.type: dobot`, `arm.port: COM3`, `velocity_ratio: 30` |
| `src/perception/hikvision_camera.py` | **新建** — 海康相机 SDK 封装 |
| `src/perception/hand_eye_calibration.py` | **新建** — 手眼标定 + 视觉引导模块 |
| `scripts/calibrate_subtract.py` | **新建** — 差分法无标记标定（可用） |
| `scripts/calibrate_board.py` | **新建** — 黑白板标定（待测试） |
| `scripts/visual_servo.py` | **新建** — 视觉抓取流程 |
| `scripts/fix_sangfor_dongle.py` | **新建** — 加密狗诊断工具 |
| `config/calib_result.json` | **新建** — 标定结果（4点，0.3mm） |

---

## 五、可运行命令速查

```bash
conda activate push_grasp

# 仿真全流程
python scripts/run_task.py --layout 1 --trials 1

# 实机全流程
python scripts/run_task.py --layout 1 --trials 1 --mode dobot

# 机械臂连接测试
python scripts/test_arm_connection.py --mode dobot --port COM3

# 相机采图测试
python -c "from src.perception.hikvision_camera import HikvisionCamera; c=HikvisionCamera(); c.open(0); print(c.grab().shape); c.close()"

# 差分法标定
python scripts/calibrate_subtract.py

# 黑白板标定
python scripts/calibrate_board.py

# 视觉抓取
python scripts/visual_servo.py
```

---

## 六、架构总结

```
相机 (HikvisionCamera)                机械臂 (DobotArm)
    │ grab()                              │ move_to_pose()
    ▼                                     ▼
图像增强 (CLAHE)                      execute_action()
    │                                     │
    ▼                                     ▼
物体检测 (差分/颜色)          ◄──────── 气泵 (set_suction_cup)
    │
    ▼
像素→世界 (calib_result.json)
    │
    ▼
机械臂抓取 (visual_servo.py)
```

**核心优势**: 全部自主代码，不依赖 DobotVisionStudio 和加密狗。
