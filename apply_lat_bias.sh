#!/bin/sh
# 修正系统性横向偏差：目标点整体左移 9cm（据点 8cm）
# 蓝卡路线：base_y 0.90 -> 0.98,  goal_y -0.90 -> -0.81
# 红卡路线：镜像对称
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(blue_base_x|blue_base_y|blue_goal_x|blue_goal_y|red_base_x|red_base_y|red_goal_x|red_goal_y)=' beacon_race.cfg

sed -i 's/^blue_base_y=.*/blue_base_y=0.98/' beacon_race.cfg
sed -i 's/^blue_goal_y=.*/blue_goal_y=-0.81/' beacon_race.cfg
sed -i 's/^red_base_y=.*/red_base_y=-0.98/'   beacon_race.cfg
sed -i 's/^red_goal_y=.*/red_goal_y=0.81/'    beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(blue_base_x|blue_base_y|blue_goal_x|blue_goal_y|red_base_x|red_base_y|red_goal_x|red_goal_y)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"