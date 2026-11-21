#!/usr/bin/env bash
echo "=== deskew 统计行 ==="
grep -a "TF ok=" /tmp/gimbal.log | tail -3
echo "=== TF 失败告警 ==="
grep -a "TF 查询失败" /tmp/gimbal.log | head -3
echo "=== 最近几条 deskew 日志 ==="
grep -a "deskew_node" /tmp/gimbal.log | tail -4
echo "=== sim 启动行 ==="
grep -a "grid_sim v1.1" /tmp/gimbal.log | tail -1
