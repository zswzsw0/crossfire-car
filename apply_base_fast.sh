#!/bin/sh
# 减少进据点的运行时间：pos_p 0.8 -> 1.2（更快接近）、base_wait_sec 2.0 -> 1.0
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^(pos_p|base_wait_sec|vel_x_odom)=' beacon_race.cfg

sed -i 's/^pos_p=.*/pos_p=1.2/'              beacon_race.cfg
sed -i 's/^base_wait_sec=.*/base_wait_sec=1.0/' beacon_race.cfg

echo "=== 改后 ==="
grep -E '^(pos_p|base_wait_sec|vel_x_odom|vel_x_vision)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"