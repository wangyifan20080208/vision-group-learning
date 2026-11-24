#!/usr/bin/env bash
set +u
echo "=== 网络检查 ==="
timeout 8 ping -c2 -W2 archive.ubuntu.com 2>&1 | tail -2
timeout 8 ping -c2 -W2 mirrors.tuna.tsinghua.edu.cn 2>&1 | tail -2

echo
echo "=== apt 源 ==="
grep -hE "^deb " /etc/apt/sources.list 2>/dev/null | head -3
ls /etc/apt/sources.list.d/ 2>/dev/null | head -5

echo
echo "=== 尝试安装 CycloneDDS（显示错误）==="
sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp 2>&1 | tail -12
