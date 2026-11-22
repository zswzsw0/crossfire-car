#!/bin/sh
# 分区提速（路径 A）：只改 cfg，不重新编译
set -e
cd /home/root
TS=$(date +%Y%m%d_%H%M%S)

echo "=== 1. 停程序并备份 ==="
pkill -x beaconRace 2>/dev/null || true
pkill -f "sleep 86400" 2>/dev/null || true
sleep 1
cp -p beacon_race.cfg "beacon_race.cfg.bak.zone.$TS"
echo "  备份 -> beacon_race.cfg.bak.zone.$TS"

echo "=== 2. 改前 ==="
grep -E '^(vel_x_vision|near_goal_dist|vel_z_max|goal_pos_tol|goal_yaw_tol)=' beacon_race.cfg

echo "=== 3. 应用分区提速 ==="
sed -i 's/^vel_x_vision=.*/vel_x_vision=0.90/'  beacon_race.cfg
sed -i 's/^near_goal_dist=.*/near_goal_dist=2.40/' beacon_race.cfg
sed -i 's/^vel_z_max=.*/vel_z_max=0.70/'        beacon_race.cfg

echo "=== 4. 改后 ==="
grep -E '^(vel_x_vision|vel_x_odom|near_goal_dist|vel_z_max|goal_pos_tol|goal_yaw_tol)=' beacon_race.cfg
echo "  配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"

echo "=== 5. 二进制（应仍是 v1.4 78991d27，未重编译）==="
md5sum beaconRace

echo "=== 6. 启动 --no-drive 验证 ==="
sh /home/root/br_ctl.sh start 2>&1 | grep -E 'PROC|beaconRace|CAM'
echo "--- 启动日志 ---"
head -9 /home/root/run_logs/br_run.log | tail -2