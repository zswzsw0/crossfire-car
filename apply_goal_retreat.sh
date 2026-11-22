#!/bin/sh
# 第一批（纯配置）：目标点后撤 + 容差放宽
set -e
cd /home/root
TS=$(date +%Y%m%d_%H%M%S)

echo "=== 1. 停程序并备份配置 ==="
pkill -x beaconRace 2>/dev/null || true
pkill -f "sleep 86400" 2>/dev/null || true
sleep 1
cp -p beacon_race.cfg "beacon_race.cfg.bak.goal690.$TS"
echo "  备份 -> beacon_race.cfg.bak.goal690.$TS"

echo "=== 2. 改前 ==="
grep -E '^(blue_goal_x|red_goal_x|goal_pos_tol|goal_yaw_tol)=' beacon_race.cfg

echo "=== 3. 应用改动 ==="
sed -i 's/^blue_goal_x=.*/blue_goal_x=6.90/' beacon_race.cfg
sed -i 's/^red_goal_x=.*/red_goal_x=6.90/'  beacon_race.cfg
sed -i 's/^goal_pos_tol=.*/goal_pos_tol=0.10/' beacon_race.cfg

echo "=== 4. 改后 ==="
grep -E '^(blue_base_x|blue_base_y|blue_goal_x|blue_goal_y|red_base_x|red_base_y|red_goal_x|red_goal_y|goal_pos_tol|goal_yaw_tol)=' beacon_race.cfg
echo "  配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"

echo "=== 5. 二进制（未重编译）==="
md5sum beaconRace

echo "=== 6. 启动 --no-drive 验证 ==="
sh /home/root/br_ctl.sh start 2>&1 | grep -E 'PROC|beaconRace|CAM'
echo "--- 启动日志 ---"
head -9 /tmp/br_run.log | tail -3