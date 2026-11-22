#!/bin/sh
# 从 v1.5 回退到已验证的 v1.4
set -e
cd /home/root
TS=$(date +%Y%m%d_%H%M%S)

echo "=== 1. 停程序 ==="
pkill -x beaconRace 2>/dev/null || true
pkill -f "sleep 86400" 2>/dev/null || true
sleep 1

echo "=== 2. 备份 v1.5 现状 ==="
[ -f beaconRace ] && cp -p beaconRace "beaconRace.v15.$TS" && echo "  beaconRace -> beaconRace.v15.$TS"
[ -f beacon_race.cfg ] && cp -p beacon_race.cfg "beacon_race.cfg.v15.$TS" && echo "  cfg -> beacon_race.cfg.v15.$TS"

echo "=== 3. 恢复 v1.4 二进制 ==="
SRC=/home/root/beaconRace.bak.20210324_110317
if [ "$(md5sum "$SRC" | cut -d' ' -f1)" != "78991d2741226647f931ce7a44c9335f" ]; then
    echo "  错误：备份 md5 不符"; exit 1
fi
cp -p "$SRC" beaconRace
chmod +x beaconRace

echo "=== 4. 恢复 v1.4 配置 ==="
CFG=/home/root/beacon_race.cfg.v14restore
if [ -f "$CFG" ]; then
    mv -f "$CFG" beacon_race.cfg
    chmod 644 beacon_race.cfg
else
    echo "  错误：没有上传 v1.4 配置"; exit 1
fi

echo "=== 5. 校验 ==="
ls -l beaconRace beacon_race.cfg
echo "--- md5 ---"
md5sum beaconRace
echo "--- 速度参数 ---"
grep -E '^(vel_x_vision|vel_x_odom|vel_z_max|goal_pos_tol|goal_yaw_tol|near_goal_dist)=' beacon_race.cfg
echo "--- 配置项数 ---"
grep -c '^[a-z_]*=' beacon_race.cfg