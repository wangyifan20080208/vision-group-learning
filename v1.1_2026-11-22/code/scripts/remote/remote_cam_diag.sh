#!/usr/bin/env bash
# 诊断相机打不开的原因
set +u
echo "=== 谁在占用相机 ==="
ps -eo pid,comm,args --sort=-pcpu 2>/dev/null | grep -iE "Mv|hik|MVS|logserver" | grep -v grep | head -10

echo
echo "=== 相关进程 ==="
pgrep -a -f "MvLogServer|MvViewer|MVS" 2>/dev/null | head -5 || echo "无"

echo
echo "=== 内核日志（最近 USB 相关）==="
dmesg 2>/dev/null | tail -20 | grep -iE "usb|xhci" | tail -8 || echo "需要 sudo"

echo
echo "=== USB 设备详情 ==="
lsusb -v -d 2bdf:0001 2>/dev/null | grep -iE "bcdUSB|MaxPower|Speed|iProduct|bNumEndpoints" | head -8

echo
echo "=== 是否被其他程序锁定 ==="
for p in /dev/bus/usb/002/004; do
  if command -v fuser >/dev/null; then
    echo -n "$p: "; sudo fuser -v "$p" 2>&1 | head -3 || echo "空闲"
  fi
done

echo
echo "=== 用官方样例试（C++/python 官方 demo）==="
ls /opt/MVS/Samples/64/ 2>/dev/null

echo
echo "=== DISPLAY ==="
echo "DISPLAY=$DISPLAY"

echo
echo "=== 重试打开（设置 DISPLAY，兼容 X）==="
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
export DISPLAY=${DISPLAY:-:0}
cd /opt/MVS/Samples/64/Python/MvImport || exit 1
timeout 60 python3 - <<'PY'
import os, sys
sys.path.append(os.getcwd())
from MvCameraControl_class import *

MvCamera.MV_CC_Initialize()
dl = MV_CC_DEVICE_INFO_LIST()
MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, dl)
info = cast(dl.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
cam = MvCamera()
print("CreateHandle:", cam.MV_CC_CreateHandle(info))
r = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
print("OpenDevice  :", r, hex(r) if r else "")
if r == 0:
    st = MVCC_INTVALUE_EX()
    cam.MV_CC_GetIntValueEx("Width", st)
    print("  宽度", st.nCurValue)
    cam.MV_CC_GetIntValueEx("Height", st)
    print("  高度", st.nCurValue)
    # 读像素格式
    st2 = MVCC_ENUMVALUE()
    if cam.MV_CC_GetEnumValue("PixelFormat", st2) == 0:
        print("  像素格式", st2.nCurValue)
    # 读曝光
    fv = MVCC_FLOATVALUE()
    if cam.MV_CC_GetFloatValue("ExposureTime", fv) == 0:
        print(f"  曝光 {fv.fCurValue:.1f} us  范围 {fv.fMin:.1f}~{fv.fMax:.1f}")
    cam.MV_CC_CloseDevice()
cam.MV_CC_DestroyHandle()
MvCamera.MV_CC_Finalize()
PY
