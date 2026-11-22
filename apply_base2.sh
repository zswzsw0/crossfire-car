#!/bin/sh
# 据点停留 3.0s -> 2.0s（起点到据点的时间略减，速度不变）
set -e
cd /home/root

echo "=== 改前 ==="
grep -E '^base_wait_sec=' beacon_race.cfg
sed -i 's/^base_wait_sec=.*/base_wait_sec=2.0/' beacon_race.cfg
echo "=== 改后 ==="
grep -E '^(base_wait_sec|vel_x_odom|vel_x_vision)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"