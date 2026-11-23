#!/bin/sh
# 对比实验：关闭图传(--no-stream) 看帧率是否提升
cd /home/root || exit 1

sh /home/root/br_ctl.sh stop >/dev/null 2>&1
sleep 2

echo "=== 以 --no-stream 启动（底盘默认是开着的），采集 15 秒 ==="
setsid sh -c "sleep 60 | stdbuf -oL ./beaconRace --no-stream > /tmp/fps_nostream.log 2>&1" \
    < /dev/null > /dev/null 2>&1 &
sleep 15

echo "--- 帧率分布 ---"
grep -o 'fps=[0-9]*' /tmp/fps_nostream.log | sort | uniq -c
echo "--- 平均 ---"
grep -o 'fps=[0-9]*' /tmp/fps_nostream.log | cut -d= -f2 | awk '{s+=$1;n++} END{if(n>0) printf "  %.1f fps（%d 个样本）\n", s/n, n; else print "  无样本"}'
echo "--- 底盘帧率 ---"
grep -o 'sf=[0-9]*' /tmp/fps_nostream.log | cut -d= -f2 | awk '{s+=$1;n++} END{if(n>0) printf "  %.0f 帧/秒\n", s/n}'

sh /home/root/br_ctl.sh stop >/dev/null 2>&1