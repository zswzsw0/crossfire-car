#!/bin/sh
# 按历史平均落点补偿瞄准点（消除系统性偏移）
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(blue_goal_y|red_goal_y)=' beacon_race.cfg

sed -i 's/^blue_goal_y=.*/blue_goal_y=-0.81/' beacon_race.cfg
sed -i 's/^red_goal_y=.*/red_goal_y=0.78/'   beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(blue_base_y|blue_goal_y|red_base_y|red_goal_y)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"