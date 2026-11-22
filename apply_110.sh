#!/bin/sh
# 外圈速度 1.63x（0.675 基准），终点区参数一律不动
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(vel_x_vision|vel_x_odom|vel_z_max|near_goal_dist|goal_pos_tol|blue_goal_x)=' beacon_race.cfg

sed -i 's/^vel_x_vision=.*/vel_x_vision=1.10/' beacon_race.cfg
sed -i 's/^vel_x_odom=.*/vel_x_odom=2.20/'     beacon_race.cfg
sed -i 's/^vel_z_max=.*/vel_z_max=0.90/'       beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(vel_x_vision|vel_x_odom|vel_z_max|near_goal_dist|goal_pos_tol|goal_yaw_tol|blue_goal_x|red_goal_x)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"