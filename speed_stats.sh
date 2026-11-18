#!/bin/sh
# 从运行日志里统计实际速度分布
L=/tmp/br_run.log
echo "=== 配置里的速度参数 ==="
grep -E '^(vel_x_odom|vel_x_vision|vel_z_max|yaw_p|pos_p|vis_wz_gain|odom_blend|near_goal_dist|goal_pos_tol|base_wait_sec|red_avoid_wz|red_avoid_vx|blue_avoid_wz|blue_avoid_vx)=' /home/root/beacon_race.cfg

echo
echo "=== 日志中 vx 的分布（NAV_GOAL 段）==="
grep '^NAV_GOAL' $L | grep -o 'vx=[-0-9.]*' | cut -d= -f2 | sort -n | uniq -c | sort -rn | head -12

echo
echo "=== 日志中 wz 的绝对值分布（NAV_GOAL 段）==="
grep '^NAV_GOAL' $L | grep -o 'wz=[-0-9.]*' | cut -d= -f2 | tr -d '-' | sort -n | uniq -c | sort -rn | head -10

echo
echo "=== vx / wz 极值 ==="
grep '^NAV_GOAL' $L | grep -o 'vx=[-0-9.]*' | cut -d= -f2 | awk 'NR==1{mn=$1;mx=$1;n=0;s=0} {n++; s+=$1; if($1<mn)mn=$1; if($1>mx)mx=$1} END{printf "  vx: 最小 %.3f  最大 %.3f  平均 %.3f  采样 %d 次\n", mn, mx, s/n, n}'
grep '^NAV_GOAL' $L | grep -o 'wz=[-0-9.]*' | cut -d= -f2 | awk 'NR==1{mn=$1;mx=$1;n=0;s=0} {v=$1<0?-$1:$1; n++; s+=v; if(v>mx)mx=v; if(v<mn)mn=v} END{printf "  |wz|: 最小 %.3f  最大 %.3f  平均 %.3f  采样 %d 次\n", mn, mx, s/n, n}'
