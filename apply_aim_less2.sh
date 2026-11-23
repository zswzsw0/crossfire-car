#!/bin/sh
# 再次减小瞄准点的内移幅度（大约再减一半）
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(blue_goal_y|red_goal_y)=' beacon_race.cfg

sed -i 's/^blue_goal_y=.*/blue_goal_y=-0.88/' beacon_race.cfg
sed -i 's/^red_goal_y=.*/red_goal_y=0.81/'   beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(blue_goal_y|red_goal_y)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"