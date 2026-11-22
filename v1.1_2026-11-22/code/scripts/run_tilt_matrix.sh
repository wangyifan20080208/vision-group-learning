#!/usr/bin/env bash
# 四个倾角各跑一遍，结果汇总到 /tmp/tilt_matrix.txt
OUT=/tmp/tilt_matrix.txt
: > "$OUT"
DIR=/mnt/d/dshworkspace/rm_nav_sim/maps_2027

for A in 15 20 30 45; do
  echo "=========================================================" | tee -a "$OUT"
  bash /mnt/d/dshworkspace/scripts/cov_scenario.sh 360 "$A" 0.15 0.275 \
       "$DIR/cov_tilt_${A}.png" 2>&1 | tee -a "$OUT"
  echo | tee -a "$OUT"
done

echo "全部完成 → $OUT"
