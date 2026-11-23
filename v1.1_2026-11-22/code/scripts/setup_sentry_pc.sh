#!/usr/bin/env bash
# ============================================================
# 哨兵小电脑 一键配置
#   目标：从一台刚装好 Ubuntu 22.04 的机器，到能跑导航仿真 + 自瞄仿真
#
# 用法：
#   bash setup_sentry_pc.sh            # 完整配置
#   bash setup_sentry_pc.sh --check    # 只检查，不安装
#
# 特性：可重复运行（每一步都先检查，已装的跳过）
# ============================================================
set +u          # ROS 的 setup.bash 与 set -u 冲突

RED='\033[1;31m'; GRN='\033[1;32m'; YEL='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GRN}[OK]${NC}   $*"; }
warn() { echo -e "${YEL}[跳过]${NC} $*"; }
err()  { echo -e "${RED}[失败]${NC} $*"; }
step() { echo; echo -e "${YEL}==== $* ====${NC}"; }

CHECK_ONLY=0
[ "$1" = "--check" ] && CHECK_ONLY=1

WS="$HOME/rm_ws/nav_sim_ws"
REPO="$HOME/vision-group-learning"
MIRROR="https://ghfast.top/https://github.com"

# ---------------------------------------------------------------- 0
step "0. 环境检查"
if ! grep -q "22.04" /etc/os-release 2>/dev/null; then
  err "不是 Ubuntu 22.04：$(lsb_release -ds 2>/dev/null)"
  err "ROS 2 Humble 只支持 22.04，请先重装。"
  exit 1
fi
ok "Ubuntu 22.04"

FREE=$(df -BG / | awk 'NR==2 {gsub("G","",$4); print $4}')
[ "${FREE:-0}" -lt 20 ] && warn "磁盘只剩 ${FREE}GB（建议 20GB+）" || ok "磁盘剩余 ${FREE}GB"
ok "内存 $(awk '/MemTotal/ {printf "%.1f", $2/1024/1024}' /proc/meminfo) GB"

if lsusb 2>/dev/null | grep -qi hikrobot; then
  ok "检测到海康相机"
else
  warn "未检测到海康相机（不影响本脚本，但自瞄需要它）"
fi

if [ "$CHECK_ONLY" = "1" ]; then
  echo
  echo "以下为当前状态："
  echo "  ROS 2        : $([ -f /opt/ros/humble/setup.bash ] && echo 已装 || echo 未装)"
  echo "  ROS 包数     : $(bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; ros2 pkg list 2>/dev/null | wc -l')"
  echo "  工作区       : $([ -d $WS/install ] && echo 已编译 || echo 未编译)"
  echo "  代码仓库     : $([ -d $REPO ] && echo 已clone || echo 未clone)"
  echo "  PlugYourRM   : $([ -d $HOME/PlugYourRM ] && echo 已clone || echo 未clone)"
  echo "  中文字体     : $(fc-list :lang=zh 2>/dev/null | wc -l) 个"
  exit 0
fi

# ---------------------------------------------------------------- 1
step "1. 基础工具"
sudo apt update -qq
sudo apt install -y -qq build-essential cmake git curl wget vim \
    python3-pip python3-venv net-tools htop lsb-release gnupg \
    usbutils tree > /dev/null 2>&1
ok "基础工具完成"

# ---------------------------------------------------------------- 2
step "2. ROS 2 Humble"
if [ -f /opt/ros/humble/setup.bash ]; then
  warn "已安装"
else
  sudo apt install -y -qq software-properties-common
  sudo add-apt-repository universe -y > /dev/null 2>&1
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null \
    || sudo curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update -qq
  sudo apt install -y ros-humble-desktop || { err "ROS 安装失败"; exit 1; }
  ok "ROS 2 Humble 安装完成"
fi

source /opt/ros/humble/setup.bash
grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ok "ROS 包数 $(ros2 pkg list 2>/dev/null | wc -l)"

# ---------------------------------------------------------------- 3
step "3. Nav2 与工具"
sudo apt install -y -qq \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-tf2-tools \
  ros-humble-xacro ros-humble-joint-state-publisher \
  python3-colcon-common-extensions > /dev/null 2>&1
ok "Nav2 完成"

# ---------------------------------------------------------------- 4
step "4. Python 依赖"
pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple > /dev/null 2>&1
pip3 install -q numpy scipy pyyaml pillow matplotlib pygame 2>&1 | tail -2
python3 -c "import numpy,yaml,scipy,pygame,PIL;print('  依赖 OK')" \
  && ok "Python 依赖完成" || err "Python 依赖有缺"

# ---------------------------------------------------------------- 5
step "5. 中文字体"
# 原生 Ubuntu 没有 /mnt/c，用 apt 装文泉驿
if fc-list :lang=zh 2>/dev/null | grep -qi "wenquanyi"; then
  warn "已有中文字体"
else
  sudo apt install -y -qq fonts-wqy-zenhei fonts-wqy-microhei > /dev/null 2>&1
  fc-cache -f > /dev/null 2>&1
  ok "已安装文泉驿正黑"
fi
ok "中文字体 $(fc-list :lang=zh 2>/dev/null | wc -l) 个"

# ---------------------------------------------------------------- 6
step "6. 代码仓库"
if [ -d "$REPO/.git" ]; then
  cd "$REPO" && git pull -q 2>/dev/null && ok "已更新"
else
  cd "$HOME"
  git clone --depth 1 "$MIRROR/wangyifan20080208/vision-group-learning.git" 2>&1 | tail -2
  [ -d "$REPO" ] && ok "已克隆" || { err "克隆失败（换镜像前缀再试）"; }
fi

# ---------------------------------------------------------------- 7
step "7. 编译工作区"
mkdir -p "$WS/src" "$WS/maps" "$WS/config"
if [ -d "$REPO/v1.1_2026-11-22/code/rm_nav_sim" ]; then
  rsync -a --delete "$REPO/v1.1_2026-11-22/code/rm_nav_sim/" "$WS/src/rm_nav_sim/" 2>/dev/null \
    || cp -r "$REPO/v1.1_2026-11-22/code/rm_nav_sim/." "$WS/src/rm_nav_sim/"
  ok "代码已同步"
fi
[ -d "$REPO/v1.1_2026-11-22/maps/rmul2027" ] && \
  cp -r "$REPO/v1.1_2026-11-22/maps/rmul2027" "$WS/maps/" 2>/dev/null && ok "地图已复制"
[ -f "$REPO/v1.1_2026-11-22/code/rm_nav_sim/config/nav2_params_2027.yaml" ] && \
  cp "$REPO/v1.1_2026-11-22/code/rm_nav_sim/config/nav2_params_2027.yaml" \
     "$WS/config/nav2_params_2027.yaml" && ok "Nav2 参数已复制"

# omni 控制器插件（第三方，让全向底盘能真正横移）
if [ ! -d "$WS/src/pb_omni_pid_pursuit_controller" ]; then
  cd "$WS/src"
  git clone -q --depth 1 -b humble \
    "$MIRROR/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller.git" 2>&1 | tail -1
fi
[ -d "$WS/src/pb_omni_pid_pursuit_controller" ] && ok "omni 控制器已就位"

cd "$WS"
colcon build --symlink-install --parallel-workers 4 2>&1 | tail -4
[ -d "$WS/install" ] && ok "编译完成" || err "编译失败"
grep -q "rm_ws/nav_sim_ws/install/setup.bash" ~/.bashrc || \
  echo "source $WS/install/setup.bash" >> ~/.bashrc

# ---------------------------------------------------------------- 8
step "8. 自瞄仿真 PlugYourRM"
if [ -d "$HOME/PlugYourRM/.git" ]; then
  warn "已存在"
else
  cd "$HOME"
  git clone -q --depth 1 "$MIRROR/Jacob123-p/PlugYourRM.git" 2>&1 | tail -1
fi

if [ -d "$HOME/PlugYourRM" ]; then
  cd "$HOME/PlugYourRM"
  pip3 install -q -e . 2>&1 | tail -1

  # 字体候选表里全是 Windows 字体名，补上 Linux 的
  if ! grep -q "wenquanyizenhei" sim/render/resources.py 2>/dev/null; then
    sed -i 's/_CJK_FONT_CANDIDATES = ("microsoftyahei", "simhei", "simsun", "dengxian")/_CJK_FONT_CANDIDATES = ("wenquanyizenhei", "wenquanyimicrohei", "notosanscjksc", "microsoftyahei", "simhei", "simsun", "dengxian")/' \
      sim/render/resources.py
    grep -q "wenquanyizenhei" sim/render/resources.py && ok "字体候选表已补 Linux 字体"
  else
    warn "字体补丁已打"
  fi

  # 自瞄方案（我写的位姿解算 + 跟踪器）
  SCHEME="$REPO/v1.1_2026-11-22/code/aim_scheme"
  if [ -d "$SCHEME" ]; then
    cp "$SCHEME/your_pose_solver.py" "$SCHEME/your_tracker.py" \
       custom/schemes/your_scheme/ 2>/dev/null && ok "自瞄方案已安装"
  fi
  ok "PlugYourRM 就位"
fi

# ---------------------------------------------------------------- 9
step "9. 验证"
source ~/.bashrc 2>/dev/null

echo "  包检查:"
ros2 pkg executables rm_nav_sim 2>/dev/null | sed 's/^/    /'
echo "  地图检查:"
ls "$WS/maps/rmul2027/" 2>/dev/null | sed 's/^/    /'
echo "  字体检查:"
python3 -c "import pygame;pygame.init();p=pygame.font.match_font('wenquanyizenhei');print('    match_font ->', p);pygame.quit()" 2>/dev/null | tail -1

step "完成"
echo "下一步："
echo "  1. source ~/.bashrc"
echo "  2. 导航仿真:  bash $REPO/v1.1_2026-11-22/code/scripts/run_sentry_demo.sh"
echo "  3. 自瞄仿真:  cd ~/PlugYourRM && python3 main.py"
