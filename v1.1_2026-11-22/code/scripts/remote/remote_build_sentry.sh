#!/usr/bin/env bash
# 编译 sentry_robot 并修复 libexec 目录。
#
# 背景：本机 colcon 把 ament_python 的 console_scripts 装到了 install/<pkg>/bin/，
# 而 `ros2 launch` 只认 install/<pkg>/lib/<pkg>/，于是报
#   "libexec directory ... does not exist"
# 这里编译后自动补一层软链，两种布局都能跑。
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash" 2>/dev/null
WS="$HOME/rm_ws/sentry_ws"
PKG=sentry_robot

cd "$WS" || exit 1
echo "=== 编译 $PKG ==="
colcon build --symlink-install --packages-select "$PKG" 2>&1 | tail -4

LIBEXEC="$WS/install/$PKG/lib/$PKG"
BINDIR="$WS/install/$PKG/bin"
# 每次都重建，且必须用**绝对路径**——
# 用相对路径会得到 `-> install/<pkg>/bin/xxx` 这种断链（踩过）。
mkdir -p "$LIBEXEC"
for f in "$BINDIR"/*; do
  [ -f "$f" ] || continue
  ln -sfn "$f" "$LIBEXEC/$(basename "$f")"
done
echo "=== libexec 软链 ==="
ls -la "$LIBEXEC" | tail -n +2 | sed 's/^/  /'

echo
echo "=== 结果 ==="
ls "$LIBEXEC" 2>/dev/null | sed 's/^/  /'
source "$WS/install/setup.bash"
ros2 pkg executables "$PKG" 2>/dev/null | sed 's/^/  /'
grep -q "sentry_ws/install/setup.bash" ~/.bashrc || \
  echo "source $WS/install/setup.bash" >> ~/.bashrc
