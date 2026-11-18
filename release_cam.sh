#!/bin/sh
# 找出谁占着摄像头，然后释放
echo "=== /dev/video* ==="
ls -l /dev/video* 2>/dev/null || echo "no /dev/video*"

echo "=== 占用摄像头的进程（扫 /proc/*/fd）==="
for fd in /proc/[0-9]*/fd/*; do
  tgt=$(readlink "$fd" 2>/dev/null) || continue
  case "$tgt" in
    /dev/video*)
      pid=$(echo "$fd" | cut -d/ -f3)
      cmd=$(tr -d '\0' < /proc/$pid/cmdline 2>/dev/null)
      echo "PID $pid  $tgt  <$cmd>"
      ;;
  esac
done

echo "=== beaconRace / 相关进程 ==="
ps -A 2>/dev/null | grep -Ei 'beacon|sleep 86400' | grep -v grep

echo "=== 结束 beaconRace ==="
pkill -x beaconRace && echo "sent TERM to beaconRace" || echo "beaconRace not running"
sleep 2
pkill -9 -x beaconRace 2>/dev/null
pkill -f "sleep 86400" 2>/dev/null
sleep 1

echo "=== 释放后复查 ==="
ps -A 2>/dev/null | grep -Ei 'beacon' | grep -v grep || echo "no beaconRace process"
echo "--- 9999 端口 ---"
netstat -ltn 2>/dev/null | grep 9999 || echo "9999 已关闭"
echo "--- 仍占用摄像头的进程 ---"
busy=0
for fd in /proc/[0-9]*/fd/*; do
  tgt=$(readlink "$fd" 2>/dev/null) || continue
  case "$tgt" in
    /dev/video*) busy=1; echo "STILL HELD: $fd -> $tgt";;
  esac
done
[ "$busy" = 0 ] && echo "摄像头已无进程占用"
