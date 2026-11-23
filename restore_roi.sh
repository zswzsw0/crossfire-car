#!/bin/sh
# 视野恢复到原值
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^roi_y_top=' beacon_race.cfg

sed -i 's/^roi_y_top=.*/roi_y_top=0.45/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(roi_y_top|vel_x_vision|near_goal_dist)' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"