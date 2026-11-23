#!/bin/sh
set -e
cd /home/root
grep -E '^blue_goal_y=' beacon_race.cfg
sed -i 's/^blue_goal_y=.*/blue_goal_y=-1.15/' beacon_race.cfg
echo '--- 改后 ---'
grep -E '^(blue_goal_y|red_goal_y)=' beacon_race.cfg
