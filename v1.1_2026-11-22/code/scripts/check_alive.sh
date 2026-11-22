#!/usr/bin/env bash
echo "=== 进程 ==="
pgrep -a -f "sim_node|deskew_node" | grep -v pgrep
echo
echo "=== sim 是否崩溃 ==="
grep -a "process has died" /tmp/gimbal.log | tail -3
echo
echo "=== 异常 ==="
grep -a -oE "[A-Za-z]+Error: .*|[A-Za-z]+Exception: .*" /tmp/gimbal.log | sort -u | tail -5
echo
echo "=== deskew 最近日志 ==="
grep -a "deskew_node\]" /tmp/gimbal.log | tail -2
echo
echo "=== 日志末尾 ==="
tail -c 800 /tmp/gimbal.log
