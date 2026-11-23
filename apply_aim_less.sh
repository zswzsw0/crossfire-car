#!/bin/sh
# 稍微减少瞄准点内移的幅度（大约减半）
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(blue_goal_y|red_goal_y)=' beacon_race.cfg

sed -i 's/^blue_goal_y=.*/blue_goal_y=-0.85/' beacon_race.cfg
sed -i 's/^red_goal_y=.*/red_goal_y=0.80/'   beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(blue_goal_y|red_goal_y)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"