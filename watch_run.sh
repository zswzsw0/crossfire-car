#!/bin/sh
# 跑一轮时盯日志：等 DONE 或超时，过程中每 10 秒报一次状态
LOG=/tmp/br_run.log
MAX=${1:-150}
i=0
last=""
while [ $i -lt $MAX ]; do
    sleep 10
    i=$((i + 10))
    cur=$(tail -1 "$LOG" 2>/dev/null)
    # 状态变化才打印
    st=$(echo "$cur" | cut -c1-60)
    if [ "$st" != "$last" ]; then
        echo "[${i}s] $st"
        last="$st"
    fi
    if echo "$cur" | grep -q "DONE"; then
        echo "=== REACHED DONE at ${i}s ==="
        echo "--- 状态统计 ---"
        for s in WAIT_CARD NAV_GOAL DONE; do
            printf '  %-10s %s\n' "$s" "$(grep -c "$s" "$LOG")"
        done
        echo "--- 首个 NAV_GOAL / 最终 DONE ---"
        grep -m1 NAV_GOAL "$LOG"
        tail -1 "$LOG"
        exit 0
    fi
done
echo "=== 超时 ${MAX}s 未到 DONE，当前状态： ==="
tail -2 "$LOG"
for s in WAIT_CARD NAV_GOAL DONE; do
    printf '  %-10s %s\n' "$s" "$(grep -c "$s" "$LOG")"
done
