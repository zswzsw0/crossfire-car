#!/bin/sh
# 板子上执行：备份旧 beaconRace，换上 beaconRace.new，输出结果
# 说明：这个脚本是整份上传的，不经 PowerShell 传参，所以引号/重定向都安全。
set -e
cd /home/root

pkill -x beaconRace 2>/dev/null || true
pkill -f "sleep 86400" 2>/dev/null || true
sleep 1

if [ -f beaconRace ]; then
    TS=$(date +%Y%m%d_%H%M%S)
    cp -p beaconRace "beaconRace.bak.$TS"
    echo "backup -> /home/root/beaconRace.bak.$TS ($(stat -c %s "beaconRace.bak.$TS") bytes)"
fi

mv -f beaconRace.new beaconRace
chmod +x beaconRace
ls -l beaconRace
md5sum beaconRace
