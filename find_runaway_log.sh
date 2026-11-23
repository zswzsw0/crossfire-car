#!/bin/sh
# 找出含 PARSE 错误的历史日志，并定位那次失控的日志
cd /home/root/run_logs || exit 1

echo "=== 含 PARSE 错误的日志 ==="
for f in run_*.log; do
    n=$(grep -c 'PARSE' "$f" 2>/dev/null)
    if [ "${n:-0}" -gt 0 ]; then
        lines=$(wc -l < "$f")
        echo "  $f  PARSE=$n  lines=$lines  mtime=$(date -r "$f" '+%m-%d %H:%M' 2>/dev/null)"
    fi
done

echo ""
echo "=== 含 NAV_GOAL 且行数最少的几个（可能是失控那次）==="
for f in run_*.log; do
    if grep -q '^NAV_GOAL' "$f" 2>/dev/null; then
        echo "  $f  nav=$(grep -c '^NAV_GOAL' "$f")  mtime=$(date -r "$f" '+%m-%d %H:%M' 2>/dev/null)"
    fi
done