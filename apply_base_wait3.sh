#!/bin/sh
# 恢复据点停车 3 秒
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^base_wait_sec=' beacon_race.cfg
sed -i 's/^base_wait_sec=.*/base_wait_sec=3.0/' beacon_race.cfg
echo "=== 改后 ==="
grep -E '^(base_wait_sec|vel_x_vision|near_goal_dist|goal_pos_tol)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"