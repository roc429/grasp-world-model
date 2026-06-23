# 全部报错汇总

> 日期: 2026-06-23 | 机器: KN1GHT | 设备: Dobot + 海康相机

---

## 一、Dobot 控制 (14个)

| # | 报错 | 根因 | 修复 |
|---|------|------|------|
| 1 | `DobotDllType.py 未找到` | DLL 文件缺失 | 从魔术师资料包复制 `DobotDllType.py` 到 `src/robot/` |
| 2 | `Could not find module 'DobotDll.dll' (or one of its dependencies)` | 缺少 Qt5 + VC++ 依赖 DLL | 复制 `Qt5Core.dll` `Qt5Network.dll` `Qt5SerialPort.dll` `msvcp120.dll` `msvcr120.dll` 到项目根 |
| 3 | `SetEndEffectorGripper() got an unexpected keyword argument 'enable'` | DLL 函数签名是 `(api, enableCtrl, on, isQueued)`，代码用了 `enable=True, grip=grip` | 改为位置参数: `(self.api, True, grip, 1)` |
| 4 | `运动等待超时 (10.0s)` ×6，总耗时 63s | `isQueued=1` 的夹爪指令进入队列但夹爪不存在，阻塞全部后续指令 | 改为 `isQueued=0` (直接执行) |
| 5 | `X=-11.0 超出安全范围 (50, 300)` | Dobot 原点 X 为负值 | 扩展 `SAFE_X_RANGE` 到 `(-50, 300)` |
| 6 | `AttributeError: module 'DobotDllType' has no attribute 'GetQueuedCmdLeftSpace'` | 此版本 DLL 无此函数 | 改为基于 `GetQueuedCmdCurrentIndex` 的稳定检测 |
| 7 | 红灯（机械臂不动）| Key 开关未开，电机硬件锁死 | 拧 Key 开关到 ON |
| 8 | `运动等待超时 (10.0s)` ×9 | 差分法标定时背景采集后机械臂没归位 | 优化标定采集流程 |
| 9 | `ModuleNotFoundError: No module named 'torch'` | conda env 未装 PyTorch | `pip install torch` (清华镜像 1.2MB/s) |
| 10 | `pip install torch` 下载失败: `IncompleteRead` | 官方源网络断开 (54MB/123MB) | 换清华镜像 `-i https://pypi.tuna.tsinghua.edu.cn/simple` |
| 11 | `UnicodeEncodeError: 'gbk' codec can't encode character '⚠'` | Windows CMD 默认 GBK 编码 | `⚠` → `[WARNING]` |
| 12 | 终端中文全部乱码 `???` | GBK vs UTF-8 | `etc/conda/activate.d/encoding_fix.bat`: `chcp 65001` + `PYTHONIOENCODING=utf-8` |
| 13 | 机械臂物理不动但脚本报成功 | 前次 `run_task.py` 崩溃后 Dobot 固件状态异常 | 断电重启 Dobot |
| 14 | `ConnectDobot` 后 `GetPose` 返回虚假位姿 `(200,0,50)` | 前次正常工作的残留状态被读取 | 断电后获取真实位姿 `(-13,0,-10)` |

## 二、海康相机 SDK (8个)

| # | 报错 | 根因 | 修复 |
|---|------|------|------|
| 15 | `Could not find module 'MvCameraControl.dll'` | DLL 不在 PATH | `os.add_dll_directory(r'C:\Program Files (x86)\MVS\Applications\Win64')` |
| 16 | `ModuleNotFoundError: No module named 'CameraParams_const'` | MvImport 子目录不在 sys.path | 添加 `_MVS_MVIMPORT` 到 sys.path |
| 17 | `OpenDevice 失败: 0x80000203` | MVS 客户端占用相机（独占模式冲突） | 关掉 MVS |
| 18 | `StartGrabbing 失败: 0x80000003` | 触发模式未正确设置 | 先关 TriggerMode 再 StartGrabbing |
| 19 | 图像极暗: `range=[3,105], mean=26` | 原始传感器数据暗，MVS 显示时软件拉伸 | 用 CLAHE 增强 |
| 20 | `SetFloatValue('ExposureTime')` 返回 0 | SDK 参数设置不生效 | 未解决 — 相机不接受 SDK 曝光控制 |
| 21 | `Exposure: 1223959552us` (垃圾值) | `MV_CC_GetFloatValue` 读曝光返回错误数据 | 用 MVS 日志读真实值 (32258us) |
| 22 | `cv2.error: !_src.empty() in cvtColor` | 采图失败(frame为空) | 添加 ret 检查和错误处理 |

## 三、OpenCV 检测 (5个)

| # | 报错/现象 | 根因 | 修复 |
|---|----------|------|------|
| 23 | `findChessboardCorners` 全部尺寸返回 False | 图像太暗 + 标定板不在视野 | CLAHE 增强 + 确认标定板位置 |
| 24 | 图像中心 200×200 纯白 `max=255, mean=255, std=0` | 白色格子过曝 | 降低曝光 |
| 25 | ArUco marker 检测不到 | 手机屏幕 marker 太小/光照不足 | 未解决 |
| 26 | `cv2.circle` `Bad argument: NumPy array marked as readonly` | HikvisionCamera 返回的 frame 不可写 | `frame = frame.copy()` |
| 27 | 差分法检测: `变化区域=0, diff max=47, thresh=22.8` 但无轮廓 | 形态学操作参数过激 | 改用 `MORPH_CLOSE` 填充空洞 |

## 四、标定 (4个)

| # | 报错/现象 | 根因 | 修复 |
|---|----------|------|------|
| 28 | 4点标定: `误差 mean=0.3mm, max=0.5mm` (精度好) | — | — |
| 29 | 9点标定: `误差 mean=57mm, max=86mm` (不可用) | 边缘点像素检测不准 + 外推范围大 | 缩小标定点范围 |
| 30 | 视觉抓取: 物体像素(1803,906)→世界(52,69)mm，机械臂移过去但没抓到 | 像素(1803)在标定范围外 (x=1041~1368) | 重新标定覆盖更大范围 |
| 31 | `KeyError: 'pixel_pts'` | JSON 中没有保存原始点数据 | 修复保存逻辑 |

## 五、加密狗 (3个)

| # | 报错/现象 | 根因 | 处理 |
|---|----------|------|------|
| 32 | USB 枚举无 VID_3386 | 狗灯亮(供电通)但数据不通 | 换狗、换口、装驱动均无效 |
| 33 | `send_heartbeat failed[10057]` | 云软锁连不上授权服务器 (WSAENOTCONN) | 放弃，用自主代码替代 |
| 34 | `sm_get_status failed. LastError:0x00000000` | Senselock 服务找不到狗 | 放弃，不依赖加密狗 |

## 六、环境/系统 (3个)

| # | 报错/现象 | 根因 | 修复 |
|---|----------|------|------|
| 35 | `conda: command not found` | Git Bash 不识别 conda | 用完整路径 `C:/Users/Administrator/miniconda3/envs/push_grasp/python.exe` |
| 36 | `EOFError: EOF when reading a line` | Bash 非交互模式不支持 `input()` | 改为非交互脚本 |
| 37 | `Segmentation fault` (exit 139) | MVS SDK CloseDevice 后 DLL 异常 | 添加 try/except 保护 |

---

## 统计

| 类别 | 报错数 | 已解决 | 未解决 |
|------|--------|--------|--------|
| Dobot 控制 | 14 | 14 | 0 |
| 海康相机 SDK | 8 | 6 | 2 |
| OpenCV 检测 | 5 | 3 | 2 |
| 标定 | 4 | 2 | 2 |
| 加密狗 | 3 | 0 | 3 (放弃) |
| 环境/系统 | 3 | 3 | 0 |
| **合计** | **37** | **28** | **7** |

## 未解决问题

1. 相机 SDK 曝光控制不生效 — 需通过 MVS 手动设置后关闭 MVS 再采图
2. ArUco 标记检测 — 手机屏幕反光/尺寸问题
3. 标定覆盖范围 — 4点精度好但范围有限
4. 气泵抓取物理验证 — `set_suction_cup()` 调用成功但未验证是否吸到物体
5. 加密狗硬件 — 疑似损坏，放弃使用
6. 棋盘格检测 — 图像偏暗 + 板子尺寸小
7. MVS SDK 关闭时 segfault — 不影响功能
