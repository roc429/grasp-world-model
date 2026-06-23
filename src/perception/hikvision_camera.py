"""
海康威视 USB3 Vision 工业相机封装 (MV-CE050-30UC)

依赖: MVS 3.1.0+ (Machine Vision Software) 已安装
用法:
    cam = HikvisionCamera()
    cam.open()
    frame = cam.grab()  # numpy array (H, W, 3) RGB
    cam.close()
"""

import os
import sys
import ctypes
import numpy as np
import logging

logger = logging.getLogger(__name__)

_MVS_ROOT = r"C:\Program Files (x86)\MVS"
_MVS_BIN = os.path.join(_MVS_ROOT, "Applications", "Win64")

# 必须在 import MvCameraControl_class 之前设置 DLL 搜索路径
os.environ["PATH"] = _MVS_BIN + ";" + os.environ.get("PATH", "")
if hasattr(os, "add_dll_directory"):
    os.add_dll_directory(_MVS_BIN)

_MVS_SAMPLES = os.path.join(_MVS_ROOT, "Development", "Samples", "Python")
_MVS_MVIMPORT = os.path.join(_MVS_SAMPLES, "MvImport")
if _MVS_SAMPLES not in sys.path:
    sys.path.insert(0, _MVS_SAMPLES)
if _MVS_MVIMPORT not in sys.path:
    sys.path.insert(0, _MVS_MVIMPORT)

# 现在可以安全导入 MVS 的 Python 封装
from MvImport.MvCameraControl_class import (  # type: ignore
    MvCamera, MV_CC_DEVICE_INFO_LIST, MV_CC_DEVICE_INFO,
    MV_FRAME_OUT_INFO_EX, MVCC_INTVALUE,
    MV_USB_DEVICE, MV_GIGE_DEVICE,
    MV_ACCESS_Exclusive, MV_TRIGGER_MODE_OFF,
    PixelType_Gvsp_RGB8_Packed,
    PixelType_Gvsp_BGR8_Packed,
    PixelType_Gvsp_Mono8,
    cast, POINTER, byref, memset, c_ubyte, sizeof,
)


class HikvisionCamera:
    """海康 USB3 Vision 工业相机"""

    def __init__(self):
        self._cam = None
        self._is_open = False
        self._is_grabbing = False
        self._payload_size = 0
        self._width = 0
        self._height = 0

    # ── 枚举 ────────────────────────────────────────

    @staticmethod
    def enumerate():
        """返回 [(index, model, serial, type)]"""
        deviceList = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, deviceList)
        if ret != 0:
            return []

        devices = []
        for i in range(deviceList.nDeviceNum):
            info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if info.nTLayerType == MV_USB_DEVICE:
                usb = info.SpecialInfo.stUsb3VInfo
                name = bytes(usb.chModelName).split(b"\x00")[0].decode()
                sn = bytes(usb.chSerialNumber).split(b"\x00")[0].decode()
                devices.append((i, name, sn, "USB3"))
            elif info.nTLayerType == MV_GIGE_DEVICE:
                gige = info.SpecialInfo.stGigEInfo
                name = bytes(gige.chModelName).split(b"\x00")[0].decode()
                devices.append((i, name, "", "GigE"))
        return devices

    # ── 打开/关闭 ────────────────────────────────────

    def open(self, index: int = 0) -> bool:
        """打开相机"""
        devices = self.enumerate()
        if not devices:
            logger.error("未找到海康相机")
            return False
        if index >= len(devices):
            logger.error(f"索引 {index} 超出范围 (共 {len(devices)} 台)")
            return False

        logger.info(f"打开 [{index}]: {devices[index][1]} SN={devices[index][2]}")

        deviceList = MV_CC_DEVICE_INFO_LIST()
        MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, deviceList)

        self._cam = MvCamera()
        stDev = cast(deviceList.pDeviceInfo[index], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = self._cam.MV_CC_CreateHandle(stDev)
        if ret != 0:
            logger.error(f"CreateHandle 失败: 0x{ret:x}")
            return False

        ret = self._cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            logger.error(f"OpenDevice 失败: 0x{ret:x}")
            return False

        # 自由采集模式
        self._cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)

        # 强制设为 RGB8（如果相机不支持，回退到 BayerRG 后软件转换）
        self._cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_RGB8_Packed)

        # 读参数
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
        self._cam.MV_CC_GetIntValue("PayloadSize", stParam)
        self._payload_size = stParam.nCurValue

        self._cam.MV_CC_GetIntValue("Width", stParam)
        self._width = stParam.nCurValue
        self._cam.MV_CC_GetIntValue("Height", stParam)
        self._height = stParam.nCurValue

        logger.info(f"{self._width}x{self._height}, payload={self._payload_size}")
        self._is_open = True
        return True

    def close(self):
        """关闭相机"""
        if self._is_grabbing:
            self._cam.MV_CC_StopGrabbing()
            self._is_grabbing = False
        if self._cam is not None:
            self._cam.MV_CC_CloseDevice()
            self._cam.MV_CC_DestroyHandle()
            self._cam = None
        self._is_open = False
        logger.info("相机已关闭")

    # ── 采图 ────────────────────────────────────────

    def start(self) -> bool:
        """开始连续采集"""
        if not self._is_open:
            return False
        ret = self._cam.MV_CC_StartGrabbing()
        if ret != 0:
            logger.error(f"StartGrabbing 失败: 0x{ret:x}")
            return False
        self._is_grabbing = True
        return True

    def stop(self):
        """停止采集"""
        if self._is_grabbing:
            self._cam.MV_CC_StopGrabbing()
            self._is_grabbing = False

    def grab(self, timeout_ms: int = 3000) -> np.ndarray | None:
        """采集一帧，返回 RGB numpy array (H, W, 3) uint8"""
        if not self._is_open:
            logger.error("相机未打开")
            return None

        was_grabbing = self._is_grabbing
        if not was_grabbing:
            if not self.start():
                return None

        try:
            data_buf = (c_ubyte * self._payload_size)()
            stFrameInfo = MV_FRAME_OUT_INFO_EX()
            memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))

            ret = self._cam.MV_CC_GetOneFrameTimeout(
                byref(data_buf), self._payload_size, stFrameInfo, timeout_ms
            )

            if ret != 0:
                logger.error(f"采图失败: 0x{ret:x}")
                return None

            w, h = stFrameInfo.nWidth, stFrameInfo.nHeight
            fmt = stFrameInfo.enPixelType

            if fmt == PixelType_Gvsp_RGB8_Packed:
                img = np.frombuffer(bytes(data_buf)[:w * h * 3], dtype=np.uint8)
                img = img.reshape(h, w, 3)
            elif fmt == PixelType_Gvsp_BGR8_Packed:
                img = np.frombuffer(bytes(data_buf)[:w * h * 3], dtype=np.uint8)
                img = img.reshape(h, w, 3)[:, :, ::-1].copy()
            elif fmt == PixelType_Gvsp_Mono8:
                img = np.frombuffer(bytes(data_buf)[:w * h], dtype=np.uint8)
                img = img.reshape(h, w)
                img = np.stack([img, img, img], axis=-1)
            else:
                # Bayer 格式 → 用 OpenCV 转 RGB
                import cv2
                raw = np.frombuffer(bytes(data_buf)[:w * h], dtype=np.uint8)
                raw = raw.reshape(h, w)
                # 尝试多种 Bayer 模式
                for code in [cv2.COLOR_BayerRG2RGB, cv2.COLOR_BayerGR2RGB,
                             cv2.COLOR_BayerGB2RGB, cv2.COLOR_BayerBG2RGB]:
                    img = cv2.cvtColor(raw, code)
                    # 检查是否合理（非全绿/全紫）
                    if img.std() > 5:
                        break
                else:
                    img = cv2.cvtColor(raw, cv2.COLOR_BayerRG2RGB)

            return img

        finally:
            if not was_grabbing:
                self.stop()

    def set_exposure(self, value_us: float):
        """设置曝光时间（微秒）"""
        if self._is_open:
            self._cam.MV_CC_SetFloatValue("ExposureTime", value_us)

    def get_intrinsics(self) -> dict:
        """返回相机内参（默认值，需标定后填入准确值）"""
        return {
            "fx": 554.3, "fy": 554.3,
            "cx": self._width / 2.0, "cy": self._height / 2.0,
            "width": self._width, "height": self._height,
        }

    @property
    def resolution(self):
        return (self._width, self._height)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()


# ── 测试 ────────────────────────────────────────────

def test_hikvision():
    """测试海康相机"""
    print("=" * 50)
    print("海康相机测试")
    print("=" * 50)

    devices = HikvisionCamera.enumerate()
    if not devices:
        print("未找到海康相机")
        return False
    for idx, name, sn, t in devices:
        print(f"  [{idx}] {name}  SN={sn}  type={t}")

    cam = HikvisionCamera()
    if not cam.open(0):
        return False

    print(f"分辨率: {cam.resolution}")

    # 提高曝光
    cam.set_exposure(30000)

    frame = cam.grab()
    if frame is not None:
        print(f"采图: shape={frame.shape}, range=[{frame.min()},{frame.max()}]")
        import cv2
        cv2.imwrite(
            os.path.join(os.path.dirname(__file__),
                         "../../experiments/hikvision_test.png"),
            frame[:, :, ::-1]
        )
        print("已保存")
        cam.close()
        return True
    else:
        cam.close()
        return False


if __name__ == "__main__":
    ok = test_hikvision()
    print(f"\n结果: {'PASS' if ok else 'FAIL'}")
