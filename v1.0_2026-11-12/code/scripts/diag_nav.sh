#!/usr/bin/env bash
L=$(ls -td "$HOME"/.ros/log/*/ | head -1)
echo "logdir=$L"
echo "=== ERROR / WARN ==="
grep -hE "\[ERROR\]|\[WARN\]" "$L"/*.log 2>/dev/null \
  | sed 's/\x1b\[[0-9;]*m//g' \
  | grep -vE "Behavior Tree tick rate" \
  | sort | uniq -c | sort -rn | head -25
