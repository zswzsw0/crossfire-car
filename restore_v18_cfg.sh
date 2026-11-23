#!/bin/sh
# 把板上配置恢复成 v1.8 快照
set -e
cd /home/root

if [ ! -f beacon_race.cfg.v18restore ]; then
    echo "错误: 没找到 beacon_race.cfg.v18restore"
    exit 1
fi

cp -p beacon_race.cfg beacon_race.cfg.bak.before_v18restore
mv -f beacon_race.cfg.v18restore beacon_race.cfg
chmod 644 beacon_race.cfg

echo "=== 恢复后关键项 ==="
grep -E '^(vel_x_vision|vel_x_odom|vel_z_max|yaw_p|odom_blend|pos_p|base_wait_sec|blue_goal_y|red_goal_y)=' beacon_race.cfg
echo "配置项数: $(grep -c '^[a-z_]*=' beacon_race.cfg)"
echo "备份: beacon_race.cfg.bak.before_v18restore"