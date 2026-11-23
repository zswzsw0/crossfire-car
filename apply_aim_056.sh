#!/bin/sh
# 方案A：蓝卡瞄准点改为实测反推值 -0.56
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(blue_goal_y|red_goal_y)=' beacon_race.cfg

sed -i 's/^blue_goal_y=.*/blue_goal_y=-0.56/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(blue_goal_y|red_goal_y)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"