#!/bin/sh
echo "=== 板上 beaconRace 备份（按时间）==="
ls -t /home/root/beaconRace.bak.* 2>/dev/null | head -8 | while read f; do
    printf '%-52s %s  %s\n' "$f" "$(stat -c %s "$f")" "$(md5sum "$f" | cut -d' ' -f1)"
done
echo
echo "=== 目标: v1.4 = 78991d2741226647f931ce7a44c9335f (160616 字节) ==="
HIT=$(ls -t /home/root/beaconRace.bak.* 2>/dev/null | while read f; do
    if [ "$(md5sum "$f" | cut -d' ' -f1)" = "78991d2741226647f931ce7a44c9335f" ]; then echo "$f"; break; fi
done)
if [ -n "$HIT" ]; then echo "找到: $HIT"; else echo "未找到 v1.4 备份"; fi
echo
echo "=== v1.4 配置备份（vel_x_vision=0.675）==="
grep -l '^vel_x_vision=0.675' /home/root/beacon_race.cfg.bak.* 2>/dev/null | head -3