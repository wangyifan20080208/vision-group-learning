#!/usr/bin/env bash
# 实测海康相机：枚举 + 抓一帧
set +u
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
cd /opt/MVS/Samples/64/Python/MvImport || exit 1
echo "工作目录: $(pwd)"
ls | head -8
echo
timeout 60 python3 - <<'PY'
import os
import sys
sys.path.append(os.getcwd())
try:
    from MvCameraControl_class import *
except Exception as e:
    import traceback
    traceback.print_exc()
    raise SystemExit(1)

MvCamera.MV_CC_Initialize()
devList = MV_CC_DEVICE_INFO_LIST()
ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, devList)
print(f"枚举返回 {ret}，发现 {devList.nDeviceNum} 个设备")
if devList.nDeviceNum == 0:
    print("没有找到相机")
    raise SystemExit(0)

for i in range(devList.nDeviceNum):
    info = cast(devList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
    if info.nTLayerType == MV_USB_DEVICE:
        d = info.SpecialInfo.stUsb3VInfo
        name = "".join(chr(c) for c in d.chModelName if c != 0)
        sn = "".join(chr(c) for c in d.chSerialNumber if c != 0)
        print(f"  [{i}] USB  {name}  SN={sn}")

# 打开第一个
cam = MvCamera()
info = cast(devList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
ret = cam.MV_CC_CreateHandle(info)
print(f"创建句柄 {ret}")
ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
print(f"打开设备 {ret}")

# 取当前参数
try:
    st = MVCC_INTVALUE_EX()
    cam.MV_CC_GetIntValueEx("Width", st);  print(f"  宽度 {st.nCurValue}")
    cam.MV_CC_GetIntValueEx("Height", st); print(f"  高度 {st.nCurValue}")
    cam.MV_CC_GetFloatValue("ExposureTime", MVCC_FLOATVALUE())
except Exception as e:
    print("读参数:", e)

cam.MV_CC_CloseDevice()
cam.MV_CC_DestroyHandle()
MvCamera.MV_CC_Finalize()
print("相机可正常打开")
PY
