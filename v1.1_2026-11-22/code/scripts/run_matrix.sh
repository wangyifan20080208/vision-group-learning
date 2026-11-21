#!/usr/bin/env bash
# 跑完整验证矩阵，结果汇总到 /tmp/matrix.txt 并打印
SCRIPT=/mnt/d/dshworkspace/scripts/verify_scenario.sh
OUT=/tmp/matrix.txt
: > "$OUT"

run() {
  local tag="$1" rate="$2" dsk="$3" ego="$4" fil="$5" rng="$6"
  echo "==============================================================="
  echo "场景 $tag"
  echo "==============================================================="
  bash "$SCRIPT" "$rate" "$dsk" "$ego" "$fil" "$rng" 6 2>&1 \
    | grep -vE "^\[debug\] " | tee -a "$OUT"
  echo
}

run "A 静止基准（无旋转，应无畸变）"          0   true  true true  0.45
run "B 云台180°/s + 去畸变 ON"              180 true  true true  0.45
run "C 云台180°/s + 去畸变 OFF（对照）"       180 false true true  0.45
run "D 云台360°/s + 去畸变 ON"              360 true  true true  0.45
run "E 云台180°/s + 自车滤波 OFF（看自遮挡）" 180 true  true false 0.45

echo "全部完成，汇总在 $OUT"
