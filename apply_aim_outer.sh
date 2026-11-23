#!/bin/sh
# 两卡进营地时偏内 → 瞄准点往远离赛道中线方向移 8cm
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(blue_goal_y|red_goal_y|blue_base_y|red_base_y)=' beacon_race.cfg

sed -i 's/^blue_goal_y=.*/blue_goal_y=-0.96/' beacon_race.cfg
sed -i 's/^red_goal_y=.*/red_goal_y=0.89/'   beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(blue_goal_y|red_goal_y|blue_base_y|red_base_y)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"