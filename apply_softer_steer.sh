#!/bin/sh
# 第二步：降低转向灵敏度（红卡末段摆动的根因）
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(yaw_p|odom_blend|vis_wz_gain)=' beacon_race.cfg

sed -i 's/^yaw_p=.*/yaw_p=1.35/'           beacon_race.cfg
sed -i 's/^odom_blend=.*/odom_blend=0.42/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(yaw_p|odom_blend|vis_wz_gain)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"