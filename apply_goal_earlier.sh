#!/bin/sh
# 配套改动：把终点减速逻辑的生效距离也提前（否则 GOAL_SLOW_DIST=2.20 会与 2.20 撞车）
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(near_goal_dist|goal_pos_tol)' beacon_race.cfg

sed -i 's/^near_goal_dist=.*/near_goal_dist=2.80/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(near_goal_dist|goal_pos_tol)' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"