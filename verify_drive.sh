#!/bin/sh
echo "=== beaconRace 进程 ==="
ps -A 2>/dev/null | grep beaconRace | grep -v grep || echo "  not running"

echo "=== 各 PID 的完整命令行 ==="
for p in $(ls /proc | grep -E '^[0-9]+$'); do
  c=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)
  case "$c" in
    *beaconRace*) echo "PID $p : $c" ;;
  esac
done

echo "=== cfg 中与驱动相关的开关 ==="
grep -E '^(enable_drive|allow_force_card|use_gyro_yaw|uart|camera_id)=' /home/root/beacon_race.cfg

echo "=== 日志文件 ==="
ls -l /tmp/br_run.log
echo "--- 日志中的关键行 ---"
tr '\r' '\n' < /tmp/br_run.log | grep -a -E 'drive|WAIT_CARD|启动' | head -5
echo "--- 日志字节数 ---"
wc -c < /tmp/br_run.log
