#!/bin/sh
# 方案 A：外圈回落到 0.90 m/s（终点区参数不动）
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(vel_x_vision|vel_x_odom|vel_z_max|near_goal_dist|goal_pos_tol|blue_goal_x)=' beacon_race.cfg

sed -i 's/^vel_x_vision=.*/vel_x_vision=0.90/' beacon_race.cfg
sed -i 's/^vel_x_odom=.*/vel_x_odom=1.80/'     beacon_race.cfg
sed -i 's/^vel_z_max=.*/vel_z_max=0.73/'       beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(vel_x_vision|vel_x_odom|vel_z_max|near_goal_dist|goal_pos_tol|goal_yaw_tol|blue_goal_x|red_goal_x)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"