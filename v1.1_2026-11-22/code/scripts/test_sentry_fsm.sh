#!/usr/bin/env bash
# 哨兵全自动闭环测试：导航 + 裁判系统 + 决策状态机
#   bash test_sentry_fsm.sh [受击速率] [时长秒] [撤退阈值]
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

DPS=$(printf "%.1f" "${1:-60}")
DUR=${2:-70}
RET=$(printf "%.2f" "${3:-0.30}")
LOG=/tmp/referee.log
SL=/tmp/sentry_state.log

bash /mnt/d/dshworkspace/scripts/kill_all.sh > /dev/null 2>&1
: > "$SL"

nohup ros2 launch rm_nav_sim sentry_sim.launch.py \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  pitch_deg:=20.0 spin_dps:=0.0 \
  match_duration:=300.0 damage_per_sec:="$DPS" hp_retreat:="$RET" \
  rviz:=false > "$LOG" 2>&1 &

echo "==> 等待 Nav2 与状态机就绪"
for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose \
     && ros2 topic list 2>/dev/null | grep -qx /referee/state; then
    echo "    就绪（${i}s）"; break
  fi
  sleep 1
done
for i in $(seq 1 30); do
  if timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -q Translation; then
    echo "    TF 就绪（${i}s）"; break
  fi
  sleep 1
done
sleep 2

echo "==> 记录 ${DUR}s（受击速率 ${DPS} HP/s，撤退阈值 ${RET}）"
timeout "$DUR" ros2 topic echo /sentry/state --csv > "$SL" 2>/dev/null
echo "    记录完成"

OUT=/tmp/referee_summary.txt
grep -E "哨兵战亡|复活|虚弱|比赛结束" "$LOG" > "$OUT" 2>/dev/null

bash /mnt/d/dshworkspace/scripts/kill_all.sh > /dev/null 2>&1

python3 - "$SL" "$OUT" <<'PY'
import json
import sys
from collections import Counter

rows = []
for line in open(sys.argv[1], encoding="utf-8", errors="ignore"):
    line = line.strip()
    if not line or line.startswith("data"):
        continue
    try:
        rows.append(json.loads(line))
    except Exception:
        pass
if not rows:
    print("没采到状态数据"); raise SystemExit

print("==> 状态转移序列")
print(f"{'帧':>5} {'状态':<14}{'hp':>6} {'所在区域':<16}{'开火':>6}")
last = None
for i, r in enumerate(rows):
    st = r.get("state")
    if st != last:
        hp = r.get("hp_frac")
        hp = f"{hp*100:.0f}%" if isinstance(hp, (int, float)) else "?"
        print(f"{i:>5} {st:<14}{hp:>6} {str(r.get('zone')):<16}"
              f"{str(r.get('fire_enable')):>6}")
        last = st

seen = [r.get("state") for r in rows]
print(f"\n共 {len(rows)} 帧，状态集合: {sorted(set(seen))}")
c = Counter(seen)
print("各状态占比：" + "  ".join(f"{k}={v/len(rows)*100:.0f}%"
                              for k, v in c.most_common()))
print(f"\n战亡次数: {seen.count('DEAD') and sum(1 for i in range(1, len(seen)) if seen[i]=='DEAD' and seen[i-1]!='DEAD')}")
print("\n==> 裁判系统事件")
for line in open(sys.argv[2], encoding="utf-8", errors="ignore"):
    print("   " + line.split("] ")[-1].strip())
PY
