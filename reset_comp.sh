#!/bin/sh
# 按「成功方案」复位板子上的比赛代码：二进制 + 配置
#   - 二进制：用板上现存的 beaconRace_ok（md5 871b0dea，与成功方案里那份一致）
#   - 配置  ：用刚上传的 beacon_race.cfg.new（成功方案里的 1783 字节版本）
# 原文件先备份成 *.bak.<时间戳>，可随时回退。
set -e
cd /home/root
TS=$(date +%Y%m%d_%H%M%S)

echo "=== 1. 备份当前文件 ==="
pkill -x beaconRace 2>/dev/null || true
pkill -f "sleep 86400" 2>/dev/null || true
sleep 1
[ -f beaconRace ]     && cp -p beaconRace     "beaconRace.bak.$TS"     && echo "  beaconRace -> beaconRace.bak.$TS"
[ -f beacon_race.cfg ] && cp -p beacon_race.cfg "beacon_race.cfg.bak.$TS" && echo "  beacon_race.cfg -> beacon_race.cfg.bak.$TS"

echo "=== 2. 复位二进制（beaconRace_ok -> beaconRace）==="
if [ ! -f beaconRace_ok ]; then
    echo "  错误：板上没有 beaconRace_ok，无法复位二进制"; exit 1
fi
cp -p beaconRace_ok beaconRace
chmod +x beaconRace

echo "=== 3. 复位配置 ==="
if [ -f beacon_race.cfg.new ]; then
    mv -f beacon_race.cfg.new beacon_race.cfg
    chmod 644 beacon_race.cfg
else
    echo "  警告：没有 beacon_race.cfg.new，配置未替换"
fi

echo "=== 4. 校验 ==="
ls -l beaconRace beacon_race.cfg
echo "--- md5 ---"
md5sum beaconRace beacon_race.cfg
echo "--- 期望值 ---"
echo "  beaconRace     应为 871b0deafd151c11e44d0c280a1e9e99"
echo "  beacon_race.cfg 应为 a8b0de8d45140fcc264d67ba419175b2"
echo "--- 配置行数 ---"
grep -c '=' beacon_race.cfg || true
