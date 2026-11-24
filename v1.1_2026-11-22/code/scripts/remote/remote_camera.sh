#!/usr/bin/env bash
# 检查相机 MVS 与 python 绑定
set +u

echo "=== MVS 目录 ==="
ls /opt/MVS/ 2>/dev/null

echo
echo "=== MVS 库 ==="
ls /opt/MVS/lib/ 2>/dev/null
ls /opt/MVS/lib/64/ 2>/dev/null | head -8

echo
echo "=== MVS 示例 ==="
ls /opt/MVS/samples/ 2>/dev/null
find /opt/MVS/samples -maxdepth 3 -type d 2>/dev/null | head -10

echo
echo "=== python 绑定 (MvImport) ==="
find /opt/MVS -name "MvImport" -type d 2>/dev/null | head -3

echo
echo "=== 相机枚举测试 ==="
MVDIR=$(find /opt/MVS/samples -maxdepth 4 -type d -name "MvImport" 2>/dev/null | head -1)
echo "MvImport: $MVDIR"
if [ -n "$MVDIR" ]; then
  ls "$MVDIR" | head -6
  PYDIR=$(dirname "$MVDIR")
  echo "PYDIR=$PYDIR"
  cd "$PYDIR" || exit 1
  export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
  timeout 30 python3 - <<'PY'
import sys, os, ctypes
sys.path.append(os.getcwd())
try:
    from MvCameraControl_class import *
    print("  导入 MvCameraControl 成功")
    MvCamera.MV_CC_Initialize()
    devList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, devList)
    print(f"  枚举返回 {ret}, 发现 {devList.nDeviceNum} 个设备")
    for i in range(devList.nDeviceNum):
        info = cast(devList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        if info.nTLayerType == MV_USB_DEVICE:
            name = "".join(chr(c) for c in info.SpecialInfo.stUsb3VInfo.chModelName if c != 0)
            serial = "".join(chr(c) for c in info.SpecialInfo.stUsb3VInfo.chSerialNumber if c != 0)
            print(f"    [{i}] USB {name}  序列号 {serial}")
    MvCamera.MV_CC_Finalize()
except Exception as e:
    import traceback; traceback.print_exc()
PY
fi

echo
echo "=== USB 权限 ==="
lsusb -d 2bdf:0001
BUSNUM=$(lsusb -d 2bdf:0001 | sed 's/Bus \([0-9]*\).*/\1/')
DEVNUM=$(lsusb -d 2bdf:0001 | sed 's/.*Device \([0-9]*\).*/\1/')
echo "bus=$BUSNUM dev=$DEVNUM"
ls -l "/dev/bus/usb/$BUSNUM/$DEVNUM" 2>/dev/null

echo
echo "=== 用户在哪些组 ==="
groups

echo
echo "=== udev 规则 ==="
ls /etc/udev/rules.d/ 2>/dev/null | head -8
grep -rl "2bdf\|Hikrobot\|hikrobot" /etc/udev/rules.d/ 2>/dev/null | head -3
