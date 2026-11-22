#!/bin/sh
# 缩短据点停车等待：3.0s -> 0.5s（提高出发反应速度）
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(base_wait_sec|near_goal_dist|goal_pos_tol|vel_x_vision|vel_z_max)=' beacon_race.cfg

sed -i 's/^base_wait_sec=.*/base_wait_sec=0.5/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(base_wait_sec|near_goal_dist|goal_pos_tol|vel_x_vision|vel_z_max)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"