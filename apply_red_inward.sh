#!/bin/sh
# 红色路线额外内收：goal_y 0.81 -> 0.75（往场地中心方向再移 6cm）
# 蓝卡已验收（落地 -0.886，偏差 1.4cm），不动
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(blue_base_y|blue_goal_y|red_base_y|red_goal_y)=' beacon_race.cfg

sed -i 's/^red_goal_y=.*/red_goal_y=0.75/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(blue_base_y|blue_goal_y|red_base_y|red_goal_y)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"