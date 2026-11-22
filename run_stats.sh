#!/bin/sh
L=/home/root/run_logs/br_run.log
echo "=== 状态行数 ==="
printf '  WAIT_CARD %s\n' "$(grep -c WAIT_CARD $L)"
printf '  NAV_GOAL  %s\n' "$(grep -c NAV_GOAL $L)"
printf '  DONE      %s\n' "$(grep -c DONE $L)"
echo "=== 首个 NAV_GOAL ==="
grep -m1 NAV_GOAL $L
echo "=== 最终一行 ==="
tail -1 $L
echo "=== 位姿极值（沿路线的 y 范围）==="
grep -o 'pose=([^)]*)' $L | tr -d 'pose=()' | awk -F, 'NF==2{if($2>maxy)maxy=$2; if($2<miny)miny=$2; if($1>maxx)maxx=$1} END{printf "  x_max=%.3f  y_min=%.3f  y_max=%.3f\n", maxx, miny, maxy}'
echo "=== 运行日志里的避让/提示 ==="
grep -aiE 'avoid|cone|stuck|warn|err=' $L | tail -3
echo "=== 进程 ==="
pgrep -af '^\./beaconRace' || pgrep -af beaconRace | head -2
