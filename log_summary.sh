#!/bin/sh
L=/tmp/br_run.log
echo "=== 日志规模 ==="
wc -l < $L; wc -c < $L
echo "=== 状态迁移（首次出现）==="
grep -n -E 'beaconRace 启动|WAIT_CARD|NAV_GOAL|NAV_|DONE|卡片|card=blue|card=red|停车|避让|ERROR|fail' $L | head -20
echo "=== 各状态行数 ==="
for s in WAIT_CARD NAV_GOAL DONE; do
  n=$(grep -c "$s" $L)
  echo "  $s: $n"
done
echo "=== 最后 3 行 ==="
tail -3 $L
echo "=== 当前进程 ==="
ps -A 2>/dev/null | grep beaconRace | grep -v grep
