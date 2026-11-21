#!/usr/bin/env bash
echo "=== sim_node 崩溃（Traceback 起 30 行）==="
grep -a -A 30 "sim_node-1\] Traceback" /tmp/gimbal.log | tail -32
echo
echo "=== 所有异常类型 ==="
grep -a -oE "[A-Za-z]+Error: .*|[A-Za-z]+Exception: .*" /tmp/gimbal.log | sort -u | head -10
