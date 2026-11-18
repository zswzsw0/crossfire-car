#!/bin/sh
# 独立摄像头预览：不跑 beaconRace 也能出实时画面，方便调摄像头角度
#   sh /home/root/cam_preview.sh start | stop | status
#
# 原理：ffmpeg 从 /dev/video0 取 MJPG（-c:v copy 不转码），在 TCP:9999 吐裸 MJPEG。
#       上位机 viewer 自动识别成"预览"模式。
# 摄像头是独占设备：beaconRace 与预览不能同时用。

LOG=/tmp/cam_preview.log
PORT=9999
LOOP=/home/root/cam_preview_loop.sh

pfx() { pgrep -f "ffmpeg.*tcp://0.0.0.0:$PORT" > /dev/null 2>&1; }

case "$1" in
  start)
    if pgrep -x beaconRace > /dev/null 2>&1; then
        echo "beaconRace 正在运行，摄像头被它占用。先执行: sh /home/root/br_ctl.sh stop"
        exit 1
    fi
    pkill -f "cam_preview_loop.sh" 2>/dev/null
    pkill -f "ffmpeg.*tcp://0.0.0.0:$PORT" 2>/dev/null
    sleep 1
    : > $LOG
    setsid nohup sh "$LOOP" < /dev/null > /dev/null 2>&1 &
    sleep 3
    echo "=== 循环进程 ==="
    pgrep -af "cam_preview_loop.sh" || echo "  (未启动)"
    echo "=== ffmpeg 进程 ==="
    pgrep -af "ffmpeg.*tcp://0.0.0.0:$PORT" || echo "  (未启动)"
    echo "=== 端口 ==="
    netstat -ltn 2>/dev/null | grep ":$PORT" || echo "  $PORT 未监听"
    echo "=== 日志 ==="
    tail -4 $LOG 2>/dev/null
    ;;
  stop)
    pkill -f "cam_preview_loop.sh" 2>/dev/null && echo "预览循环已停" || echo "预览循环未运行"
    pkill -f "ffmpeg.*tcp://0.0.0.0:$PORT" 2>/dev/null && echo "ffmpeg 已停" || true
    sleep 1
    netstat -ltn 2>/dev/null | grep ":$PORT" || echo "  $PORT 已关闭"
    ;;
  status)
    echo "=== 预览 ==="
    pgrep -af "cam_preview_loop.sh" || echo "  (未运行)"
    pgrep -af "ffmpeg.*tcp://0.0.0.0:$PORT" || true
    echo "=== beaconRace ==="
    pgrep -af beaconRace | grep -v pgrep || echo "  (未运行)"
    echo "=== 摄像头占用 ==="
    for fd in /proc/[0-9]*/fd/*; do
        t=$(readlink "$fd" 2>/dev/null) || continue
        case "$t" in /dev/video*) echo "  $fd -> $t";; esac
    done
    ;;
  *)
    echo "usage: $0 start | stop | status"
    ;;
esac
