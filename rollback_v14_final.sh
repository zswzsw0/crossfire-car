#!/bin/sh
# 回退到 v1.4（= f2aa753 的代码状态，但用未损坏的源码编译的二进制 78991d27）
set -e
cd /home/root
TS=$(date +%Y%m%d_%H%M%S)

echo "=== 1. 停程序 ==="
pkill -x beaconRace 2>/dev/null || true
pkill -f "sleep 86400" 2>/dev/null || true
sleep 1

echo "=== 2. 备份当前（避让提速版）==="
[ -f beaconRace ] && cp -p beaconRace "beaconRace.avoid015.$TS" && echo "  beaconRace -> beaconRace.avoid015.$TS"
[ -f beacon_race.cfg ] && cp -p beacon_race.cfg "beacon_race.cfg.avoid015.$TS" && echo "  cfg -> beacon_race.cfg.avoid015.$TS"

echo "=== 3. 找 v1.4 二进制 (78991d27) ==="
SRC=""
for f in $(ls -t /home/root/beaconRace.bak.* /home/root/beaconRace_ok 2>/dev/null); do
    if [ "$(md5sum "$f" | cut -d' ' -f1)" = "78991d2741226647f931ce7a44c9335f" ]; then SRC="$f"; break; fi
done
if [ -z "$SRC" ]; then echo "  错误：找不到 78991d27"; exit 1; fi
echo "  找到: $SRC"

echo "=== 4. 恢复 v1.4 二进制 ==="
cp -p "$SRC" beaconRace
chmod +x beaconRace

echo "=== 5. 恢复 v1.4 配置 ==="
if [ ! -f /home/root/beacon_race.cfg.v14 ]; then
    echo "  错误：没有上传 v1.4 配置"; exit 1
fi
mv -f /home/root/beacon_race.cfg.v14 beacon_race.cfg
chmod 644 beacon_race.cfg

echo "=== 6. 校验 ==="
ls -l beaconRace beacon_race.cfg
md5sum beaconRace
echo "--- 关键参数 ---"
grep -E '^(vel_x_vision|vel_x_odom|vel_z_max|goal_pos_tol|goal_yaw_tol|near_goal_dist|blue_goal_x|red_goal_x)=' beacon_race.cfg
echo "--- 配置项数 ---"
grep -c '^[a-z_]*=' beacon_race.cfg

echo "=== 7. 启动 --no-drive 验证 ==="
sh /home/root/br_ctl.sh start 2>&1 | grep -E 'PROC|beaconRace|CAM'
echo "--- 启动日志 ---"
head -9 /home/root/run_logs/br_run.log | tail -3