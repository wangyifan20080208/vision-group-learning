#!/usr/bin/env bash
# 相机性能剖析：分辨瓶颈在「取帧 / 转换 / 发布」哪一环
set +u
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
cd /opt/MVS/Samples/64/Python/MvImport || exit 1

timeout 120 python3 - <<'PY' 2>&1 | grep -v XOpenDisplay
import os, sys, time
sys.path.append(os.getcwd())
from MvCameraControl_class import *
import numpy as np

MvCamera.MV_CC_Initialize()
dl = MV_CC_DEVICE_INFO_LIST()
MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, dl)
info = cast(dl.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
cam = MvCamera()
cam.MV_CC_CreateHandle(info)
r = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
print("OpenDevice", r)
if r: raise SystemExit(1)

cam.MV_CC_SetEnumValue("ExposureAuto", 0)
cam.MV_CC_SetEnumValue("GainAuto", 0)
cam.MV_CC_SetFloatValue("ExposureTime", 2000.0)
cam.MV_CC_SetFloatValue("Gain", 17.0)
cam.MV_CC_SetEnumValue("TriggerMode", 0)
cam.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True)

def bench(w, h, fps, n=60):
    cam.MV_CC_StopGrabbing()
    if w != 1440:
        cam.MV_CC_SetIntValue("Width", w)
        cam.MV_CC_SetIntValue("Height", h)
    cam.MV_CC_SetFloatValue("AcquisitionFrameRate", float(fps))
    cam.MV_CC_StartGrabbing()
    data = MV_FRAME_OUT()
    buf = (c_ubyte * (w * h * 3))()
    # 预热
    for _ in range(5):
        if cam.MV_CC_GetImageBuffer(data, 500) == 0:
            cam.MV_CC_FreeImageBuffer(data)
    # 1) 纯取帧
    t0 = time.time(); k = 0
    while k < n:
        if cam.MV_CC_GetImageBuffer(data, 500) == 0:
            cam.MV_CC_FreeImageBuffer(data); k += 1
    t_grab = (time.time() - t0) / n
    # 2) 取帧 + 转换
    t0 = time.time(); k = 0
    while k < n:
        if cam.MV_CC_GetImageBuffer(data, 500) == 0:
            fi = data.stFrameInfo
            conv = MV_CC_PIXEL_CONVERT_PARAM()
            conv.nWidth = fi.nWidth; conv.nHeight = fi.nHeight
            conv.pSrcData = data.pBufAddr; conv.nSrcDataLen = fi.nFrameLen
            conv.enSrcPixelType = fi.enPixelType
            conv.enDstPixelType = PixelType_Gvsp_RGB8_Packed
            conv.pDstBuffer = buf; conv.nDstBufferSize = w * h * 3
            cam.MV_CC_ConvertPixelType(conv)
            cam.MV_CC_FreeImageBuffer(data); k += 1
    t_conv = (time.time() - t0) / n
    # 3) + numpy 拷贝
    t0 = time.time(); k = 0
    while k < n:
        if cam.MV_CC_GetImageBuffer(data, 500) == 0:
            fi = data.stFrameInfo
            conv = MV_CC_PIXEL_CONVERT_PARAM()
            conv.nWidth = fi.nWidth; conv.nHeight = fi.nHeight
            conv.pSrcData = data.pBufAddr; conv.nSrcDataLen = fi.nFrameLen
            conv.enSrcPixelType = fi.enPixelType
            conv.enDstPixelType = PixelType_Gvsp_RGB8_Packed
            conv.pDstBuffer = buf; conv.nDstBufferSize = w * h * 3
            cam.MV_CC_ConvertPixelType(conv)
            arr = np.frombuffer(buf, np.uint8, w*h*3).reshape(h, w, 3)
            b = arr.tobytes()
            cam.MV_CC_FreeImageBuffer(data); k += 1
    t_np = (time.time() - t0) / n
    cam.MV_CC_StopGrabbing()
    print(f"  {w}x{h} @{fps}fps: 取帧 {t_grab*1000:6.1f}ms({1/t_grab:5.1f}fps) | "
          f"+转换 {t_conv*1000:6.1f}ms({1/t_conv:5.1f}fps) | "
          f"+numpy {t_np*1000:6.1f}ms({1/t_np:5.1f}fps)")

print("=== 性能剖析 ===")
bench(1440, 1080, 100)
bench(1024, 768, 100)
bench(720, 540, 150)

# 查询实际生效的帧率
cam.MV_CC_StartGrabbing()
fv = MVCC_FLOATVALUE()
if cam.MV_CC_GetFloatValue("AcquisitionFrameRate", fv) == 0:
    print(f"  实际设定帧率 {fv.fCurValue}")
bad = MVCC_FLOATVALUE()
if cam.MV_CC_GetFloatValue("ResultingFrameRate", bad) == 0:
    print(f"  相机实际输出 {bad.fCurValue} fps")
cam.MV_CC_StopGrabbing()
cam.MV_CC_CloseDevice(); cam.MV_CC_DestroyHandle(); MvCamera.MV_CC_Finalize()
PY
