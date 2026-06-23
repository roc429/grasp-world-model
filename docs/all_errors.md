# 实机调试全部报错一览

> 2026-06-23 · KN1GHT · Dobot + 海康 MV-CE050-30UC

---

## 总览

```
共 37 个报错
████████████████████████████████ 28 已解决 (76%)
████████ 7 未解决 (19%)
██ 2 放弃 (5%)
```

---

## 1. Dobot 机械臂 (14个 ✅ 全部解决)

### 1.1 文件缺失

| 报错 | 修复 |
|------|------|
| `DobotDllType.py 未找到` | 复制 DLL 文件到 `src/robot/` |
| `Could not find DobotDll.dll` | 复制 Qt5* + msvc* 依赖 DLL 到项目根 |

### 1.2 API 不兼容

| 报错 | 修复 |
|------|------|
| `SetEndEffectorGripper() got unexpected keyword argument 'enable'` | 位置参数: `(api, True, grip, 1)` |
| `module has no attribute 'GetQueuedCmdLeftSpace'` | 用 `GetQueuedCmdCurrentIndex` 替代 |

### 1.3 运动故障

| 症状 | 原因 | 修复 |
|------|------|------|
| 红灯、不运动 | Key 开关未开 | 拧 Key→绿灯 |
| `运动等待超时` 63s | 夹爪 `isQueued=1` 阻塞队列 | → `isQueued=0` |
| 脚本报成功但不动 | 前次崩溃残留状态 | **断电重启** |
| 位姿读数虚假 `(200,0,50)` | 读取缓存值 | 断电后读真实值 `(-13,0,-10)` |
| `X=-11 超出安全范围` | 原点 X 为负 | 扩展 `(-50, 300)` |

### 1.4 环境

| 报错 | 修复 |
|------|------|
| `No module named 'torch'` | `pip install torch` (清华镜像) |
| `⚠` emoji 报 GBK 错误 | → `[WARNING]` |
| 中文乱码 | `chcp 65001` + `PYTHONIOENCODING=utf-8` |

---

## 2. 海康相机 (8个 🔶 2个未解决)

| 报错 | 修复 |
|------|------|
| `Could not find MvCameraControl.dll` | `os.add_dll_directory()` 加路径 |
| `No module named 'CameraParams_const'` | MvImport 加到 sys.path |
| `OpenDevice 0x80000203` | **关掉 MVS** (独占冲突) |
| `StartGrabbing 0x80000003` | 先关 TriggerMode |
| 图像极暗 mean=26 | CLAHE 增强 |
| `cv2.cvtColor !_src.empty()` | 检查 ret 再处理 |
| **`SetFloatValue('ExposureTime')` 不生效** | ❌ 未解决 — 需 MVS 手动设 |
| **闭 MVS 后 SDK 偶尔 segfault** | ❌ 不影响功能 |

---

## 3. 图像检测 (5个 🔶 2个未解决)

| 尝试 | 结果 |
|------|------|
| 棋盘格 `findChessboardCorners` 全部尺寸 | ❌ 图像太暗 |
| ArUco marker (手机屏幕) | ❌ 未检测到 |
| 差分法检测末端 (原始图像) | ❌ 图像太暗 |
| 差分法 + CLAHE 增强 | ✅ 4/6 点成功 |
| 差分法 + 白纸标记 | ✅ 检测到变化 |

---

## 4. 标定 (4个 🔶 2个未解决)

| 方案 | 精度 | 状态 |
|------|------|------|
| 差分法 4点 | **0.3mm** | ✅ 可用（范围有限） |
| 差分法 9点 | **57mm** | ❌ 边缘点不准 |
| 黑白板方案 | — | ⏳ 待测试 |
| 视觉抓取实测 | 位置偏 | ❌ 坐标外推超范围 |

---

## 5. 加密狗 (3个 🚫 放弃)

| 现象 | 结论 |
|------|------|
| USB 无 VID_3386 | 灯亮但数据不通 — 硬件疑似损坏 |
| `send_heartbeat failed[10057]` | 授权服务器连不上 |
| `sm_get_status failed` | Senselock 服务找不到狗 |

→ **用自主代码替代 DobotVisionStudio，不再依赖加密狗**

---

## 6. 系统环境 (3个 ✅)

| 报错 | 修复 |
|------|------|
| `conda: command not found` | 用 Python 完整路径 |
| `EOFError` (非交互模式) | 改为非交互脚本 |
| `Segmentation fault` | try/except 保护 |

---

## 当前状态

```
✅ Dobot 实机推-抓-放         (Layout 1/2/3 全通过)
✅ 海康相机采图               (2592×1944 RGB)
✅ 手眼标定(差分法 0.3mm)    (范围有限)
⏳ 黑白板标定                (待测试)
⏳ 视觉抓取端到端            (坐标转换需优化)
🚫 加密狗                    (放弃)
```
