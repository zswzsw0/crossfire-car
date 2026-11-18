#!/bin/sh
# 虚拟机端：保证上位机窗口在跑（幂等，重复执行不会开多个窗口）
#   sh /home/mark/vm_watch.sh              默认 192.168.1.3:9999 1000x760
#   sh /home/mark/vm_watch.sh 192.168.1.3 9999 1074 900
# 已经在跑就只报状态，不会重复启动。
IP="${1:-192.168.1.3}"
PORT="${2:-9999}"
WW="${3:-1000}"
WH="${4:-760}"
LOG=/tmp/gtk_viewer.log

if pgrep -f '^python3 /home/mark/gtk_viewer.py' > /dev/null 2>&1; then
  echo "上位机窗口已在运行："
  pgrep -af '^python3 /home/mark/gtk_viewer.py'
else
  echo "启动上位机窗口 $IP:$PORT ${WW}x${WH} ..."
  DISPLAY=:0 XAUTHORITY="$HOME/.Xauthority" \
    setsid nohup python3 /home/mark/gtk_viewer.py "$IP" "$PORT" "$WW" "$WH" \
    > "$LOG" 2>&1 < /dev/null &
  sleep 4
  pgrep -af '^python3 /home/mark/gtk_viewer.py' || { echo "启动失败，看 $LOG"; tail -5 "$LOG"; exit 1; }
fi

echo "--- 最近日志 ---"
tail -5 "$LOG" 2>/dev/null
echo "--- 与板子的连接 ---"
ss -tn 2>/dev/null | grep "$IP:$PORT" || echo "（还没连上，等 beaconRace 起来会自动连）"
echo "--- 当前窗口标题 ---"
DISPLAY=:0 XAUTHORITY="$HOME/.Xauthority" xwininfo -root -tree 2>/dev/null | grep -i beaconRace | head -2
