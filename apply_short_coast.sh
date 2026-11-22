#!/bin/sh
set -e
cd /home/root
echo "=== 改前 ==="
grep -E '^(near_goal_dist|goal_pos_tol|goal_yaw_tol|blue_goal_x|red_goal_x)=' beacon_race.cfg

sed -i 's/^near_goal_dist=.*/near_goal_dist=2.20/' beacon_race.cfg
sed -i 's/^goal_pos_tol=.*/goal_pos_tol=0.14/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(near_goal_dist|goal_pos_tol|goal_yaw_tol|blue_goal_x|red_goal_x)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"

echo "=== 启动 --no-drive 验证 ==="
sh /home/root/br_ctl.sh start 2>&1 | grep -E 'PROC|beaconRace|CAM'
head -9 /home/root/run_logs/br_run.log | tail -3