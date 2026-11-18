#!/bin/sh
L=/tmp/br_run.log
echo "=== vx 分布 ==="
grep '^NAV_GOAL' $L | grep -o 'vx=[-0-9.]*' | cut -d= -f2 | sort -n | uniq -c
echo "=== |wz| 分布（前 6）==="
grep '^NAV_GOAL' $L | grep -o 'wz=[-0-9.]*' | cut -d= -f2 | tr -d '-' | sort -n | uniq -c | sort -rn | head -6
echo "=== 打满 vel_z_max(0.55) 的次数 ==="
grep '^NAV_GOAL' $L | grep -c 'wz=0.55'
echo "=== 状态行数 ==="
printf '  WAIT_CARD %s\n' "$(grep -c WAIT_CARD $L)"
printf '  NAV_GOAL  %s\n' "$(grep -c NAV_GOAL $L)"
printf '  DONE      %s\n' "$(grep -c DONE $L)"
echo "=== 最终 ==="
tail -1 $L
