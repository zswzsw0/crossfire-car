#!/bin/sh
# 红卡瞄准点外移 7cm：0.75 -> 0.82（蓝卡不动）
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(blue_goal_y|red_goal_y)=' beacon_race.cfg

sed -i 's/^red_goal_y=.*/red_goal_y=0.82/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(blue_base_y|blue_goal_y|red_base_y|red_goal_y)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"