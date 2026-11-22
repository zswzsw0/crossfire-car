#!/bin/sh
# 提高转向能力以匹配 1.43 m/s：yaw_p 1.2 -> 1.7，odom_blend 0.35 -> 0.45
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(yaw_p|odom_blend|vis_wz_gain|vel_z_max|vel_x_vision)=' beacon_race.cfg

sed -i 's/^yaw_p=.*/yaw_p=1.7/'           beacon_race.cfg
sed -i 's/^odom_blend=.*/odom_blend=0.45/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(yaw_p|odom_blend|vis_wz_gain|vel_z_max|vel_x_vision)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"