#!/usr/bin/env python3
"""海康 MV-CS016-10UC → ROS 2 Image。

自瞄对相机的三个硬要求（见 docs/方案_哨兵全自动_导航与自瞄.md）：
  ① 手动曝光、极低（超小曝光模式 1~14 µs）—— 装甲板是主动发光，压低曝光突出灯条
  ② 手动增益（这套参数下 17 dB 是上限）
  ③ 稳定时间戳 —— 延迟补偿的前提

MVS SDK 不在站点包里，靠 LD_LIBRARY_PATH 找。启动脚本会设好。
"""
from __future__ import annotations

import array
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# 让 python 找得到 MvImport（MVS 自带）
MVS_PY = "/opt/MVS/Samples/64/Python/MvImport"
if os.path.isdir(MVS_PY):
    sys.path.append(MVS_PY)

MVS_AVAILABLE = True
try:
    from MvCameraControl_class import (          # noqa: E402
        MV_ACCESS_Exclusive, MV_CC_DEVICE_INFO, MV_CC_DEVICE_INFO_LIST,
        MV_CC_PIXEL_CONVERT_PARAM, MV_FRAME_OUT, MV_USB_DEVICE, MvCamera,
        PixelType_Gvsp_BGR8_Packed, PixelType_Gvsp_RGB8_Packed, cast,
        POINTER, c_ubyte,
    )
except Exception as e:                            # noqa: BLE001
    MVS_AVAILABLE = False
    print(f"[hik_camera] 未找到 MVS SDK: {e}")


class HikCamera(Node):
    def __init__(self):
        super().__init__("hik_camera")
        d = [
            ("exposure_us", 2000.0),      # 超小曝光模式可到 1~14 µs
            ("gain_db", 17.0),            # 0~17 dB
            ("frame_rate", 100.0),        # 目标帧率（相机上限 249.1）
            ("width", 1440),
            ("height", 1080),
            ("publish_cloud", False),
            ("frame_id", "camera_optical"),
            ("image_topic", "/camera/image_raw"),
        ]
        for n, v in d:
            self.declare_parameter(n, v)
        p = self.get_parameter

        self.exposure = float(p("exposure_us").value)
        self.gain = float(p("gain_db").value)
        self.fps = float(p("frame_rate").value)
        self.width = int(p("width").value)
        self.height = int(p("height").value)
        self.frame_id = str(p("frame_id").value)

        self.pub = self.create_publisher(Image, str(p("image_topic").value), 5)
        self.cam = None
        self.data = MV_FRAME_OUT() if MVS_AVAILABLE else None
        self.buf = None
        self.conv = None
        self.n = 0
        self.t_last = time.time()

        if not MVS_AVAILABLE:
            self.get_logger().error("MVS SDK 不可用，节点空转")
            return
        if not self._open():
            self.get_logger().error("相机打开失败（是不是 MVS 图形界面还开着？）")
            return
        self.create_timer(1.0 / max(1.0, self.fps), self.grab)

    # ---------------- 打开相机 ----------------
    def _open(self) -> bool:
        MvCamera.MV_CC_Initialize()
        dl = MV_CC_DEVICE_INFO_LIST()
        MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, dl)
        if dl.nDeviceNum == 0:
            self.get_logger().error("没枚举到相机")
            return False
        info = cast(dl.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
        dev = info.SpecialInfo.stUsb3VInfo
        name = "".join(chr(c) for c in dev.chModelName if c != 0)
        sn = "".join(chr(c) for c in dev.chSerialNumber if c != 0)
        self.get_logger().info(f"发现相机 {name} SN={sn}")

        self.cam = MvCamera()
        if self.cam.MV_CC_CreateHandle(info) != 0:
            self.get_logger().error("CreateHandle 失败")
            return False
        r = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if r != 0:
            self.get_logger().error(f"OpenDevice 失败 {hex(r)}（MVS 图形界面占用？）")
            return False

        # 手动曝光/增益（关键：不要自动，否则场上光线一变就崩）
        self.cam.MV_CC_SetEnumValue("ExposureAuto", 0)
        self.cam.MV_CC_SetEnumValue("GainAuto", 0)
        self.cam.MV_CC_SetFloatValue("ExposureTime", self.exposure)
        self.cam.MV_CC_SetFloatValue("Gain", self.gain)
        self.cam.MV_CC_SetEnumValue("TriggerMode", 0)     # 连续采集
        self.cam.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True)
        self.cam.MV_CC_SetFloatValue("AcquisitionFrameRate", self.fps)

        self.cam.MV_CC_StartGrabbing()
        self.n_px = self.width * self.height
        self.buf = (c_ubyte * (self.n_px * 3))()
        self.get_logger().info(
            f"相机已开 | 曝光 {self.exposure:.0f}µs 增益 {self.gain:.0f}dB "
            f"{self.width}x{self.height} @ {self.fps:.0f}fps")
        return True

    # ---------------- 抓帧 ----------------
    def grab(self):
        if self.cam is None:
            return
        if self.cam.MV_CC_GetImageBuffer(self.data, 200) != 0:
            return
        try:
            fi = self.data.stFrameInfo
            conv = MV_CC_PIXEL_CONVERT_PARAM()
            conv.nWidth = fi.nWidth
            conv.nHeight = fi.nHeight
            conv.pSrcData = self.data.pBufAddr
            conv.nSrcDataLen = fi.nFrameLen
            conv.enSrcPixelType = fi.enPixelType
            conv.enDstPixelType = PixelType_Gvsp_RGB8_Packed
            conv.pDstBuffer = self.buf
            conv.nDstBufferSize = self.n_px * 3
            if self.cam.MV_CC_ConvertPixelType(conv) != 0:
                return
            w, h = fi.nWidth, fi.nHeight
            arr = np.frombuffer(self.buf, np.uint8, w * h * 3).reshape(h, w, 3)

            m = Image()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.frame_id
            m.height = h
            m.width = w
            m.encoding = "rgb8"
            m.is_bigendian = 0
            m.step = w * 3
            # ⚠️ 必须用 array.array('B')，不能用 bytes / bytearray。
            # 实测（1440x1080，4.67 MB）：
            #   m.data = arr.tobytes()            213 ms  ( 22 MB/s)  ← 逐字节转换
            #   m.data = array.array('B', raw)    0.1 ms  (32 GB/s)  ← 内存直拷
            # 差 2000 倍。用 bytes 会让整条链路卡在 ~4 fps。
            m.data = array.array("B", arr.tobytes())
            self.pub.publish(m)

            self.n += 1
            now = time.time()
            if now - self.t_last > 10.0:
                self.get_logger().info(
                    f"已发布 {self.n} 帧 ({(self.n/(now-self.t_last)):.1f} fps 均值)")
                self.t_last = now
        finally:
            self.cam.MV_CC_FreeImageBuffer(self.data)

    def destroy_node(self):
        if self.cam is not None:
            try:
                self.cam.MV_CC_StopGrabbing()
                self.cam.MV_CC_CloseDevice()
                self.cam.MV_CC_DestroyHandle()
                MvCamera.MV_CC_Finalize()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    n = HikCamera()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
