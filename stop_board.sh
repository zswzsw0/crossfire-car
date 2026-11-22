#!/bin/sh
# 停止 beaconRace（不停虚拟机上位机）
pkill -x beaconRace 2>/dev/null && echo "beaconRace stopped (TERM)" || echo "beaconRace not running"
sleep 1
pkill -9 -x beaconRace 2>/dev/null
pkill -f "sleep 8640" 2>/dev/null
sleep 1
echo "=== 复查 ==="
ps -A 2>/dev/null | grep beaconRace | grep -v grep || echo "  无 beaconRace 进程"
netstat -ltn 2>/dev/null | grep 9999 || echo "  9999 已关闭"
echo "=== 摄像头占用 ==="
n=0
for fd in /proc/[0-9]*/fd/*; do
  t=$(readlink "$fd" 2>/dev/null) || continue
  case "$t" in /dev/video*) echo "  $fd -> $t"; n=1;; esac
done
[ "$n" = 0 ] && echo "  无占用"
echo "=== 最后日志 ==="
tail -3 /home/root/run_logs/br_run.log 2>/dev/null