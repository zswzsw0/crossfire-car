#!/bin/sh
# 找「有 NAV 但没有 DONE」的日志 —— 那就是异常中断（失控）的那一次
cd /home/root/run_logs || exit 1

echo "=== 未正常结束（无 DONE）的日志 ==="
for f in run_*.log; do
    if grep -q '^NAV_GOAL' "$f" 2>/dev/null && ! grep -q '^DONE' "$f" 2>/dev/null; then
        echo "  $f  nav=$(grep -c '^NAV_GOAL' "$f")  lines=$(wc -l < "$f")  mtime=$(date -r "$f" '+%m-%d %H:%M' 2>/dev/null)"
    fi
done