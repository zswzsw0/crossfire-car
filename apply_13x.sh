#!/bin/sh
# 直线速度整体 x1.3：1.10 -> 1.43
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(vel_x_vision|vel_x_odom|vel_z_max|goal_pos_tol|base_wait_sec)=' beacon_race.cfg

sed -i 's/^vel_x_vision=.*/vel_x_vision=1.43/' beacon_race.cfg
sed -i 's/^vel_x_odom=.*/vel_x_odom=2.86/'     beacon_race.cfg
sed -i 's/^vel_z_max=.*/vel_z_max=1.17/'       beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(vel_x_vision|vel_x_odom|vel_z_max|goal_pos_tol|goal_yaw_tol|near_goal_dist|base_wait_sec)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"