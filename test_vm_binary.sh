#!/bin/sh
# 在板子上试跑虚拟机上交叉编译出来的 beaconRace_vm（不覆盖原程序）
cd /home/root || exit 1

echo "=== 停止现有实例，腾出摄像头 ==="
pkill -x beaconRace 2>/dev/null
pkill -f "sleep 86400" 2>/dev/null
sleep 2

echo "=== 试跑 beaconRace_vm --no-drive 8 秒 ==="
setsid sh -c "sleep 8 | ./beaconRace_vm --no-drive > /tmp/br_vm_test.log 2>&1" < /dev/null > /dev/null 2>&1 &
sleep 10

echo "=== 进程 ==="
ps -A 2>/dev/null | grep beaconRace_vm | grep -v grep || echo "  (已退出)"
echo "=== 输出 ==="
cat /tmp/br_vm_test.log
echo "=== 摄像头 ==="
for fd in /proc/[0-9]*/fd/*; do
  t=$(readlink "$fd" 2>/dev/null) || continue
  case "$t" in /dev/video*) echo "  $fd -> $t";; esac
done
echo "=== 清理测试实例 ==="
pkill -x beaconRace_vm 2>/dev/null
sleep 1
ps -A 2>/dev/null | grep beaconRace_vm | grep -v grep || echo "  已清理"
