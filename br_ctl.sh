#!/bin/sh
# beaconRace 控制器
#   start [--drive]   起主程序（默认 --no-drive）
#   stop             停主程序 + 释放摄像头 + 清 stdin 管道
#   status           看进程/端口/摄像头占用
#   cam              只列摄像头占用者
#   cam --force      杀掉所有占用 /dev/video* 的进程
cd /home/root || exit 1

# 运行日志放持久目录：/tmp 是内存盘，会被 systemd 清理，重启也会丢
LOG=/home/root/run_logs/br_run.log

holders() {   # 列出占用 /dev/video* 的 PID 和设备
  n=0
  for fd in /proc/[0-9]*/fd/*; do
    t=$(readlink "$fd" 2>/dev/null) || continue
    case "$t" in
      /dev/video*)
        pid=$(echo "$fd" | cut -d/ -f3)
        c=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null)
        echo "  CAM PID $pid  $t  <$c>"
        n=$((n+1))
        ;;
    esac
  done
  [ "$n" = 0 ] && echo "  无进程占用摄像头"
}

pids_holding_cam() {
  for fd in /proc/[0-9]*/fd/*; do
    t=$(readlink "$fd" 2>/dev/null) || continue
    case "$t" in
      /dev/video*) echo "$fd" | cut -d/ -f3 ;;
    esac
  done
}

case "$1" in
  start)
    shift
    MODE="--no-drive"
    [ "${1:-}" = "--drive" ] && MODE=""
    pkill -x beaconRace 2>/dev/null
    pkill -f "sleep 86400" 2>/dev/null
    # 摄像头独占：先把独立预览服务停掉，否则 beaconRace 打不开相机
    pkill -f "cam_preview_loop.sh" 2>/dev/null
    pkill -f "ffmpeg.*tcp://0.0.0.0:9999" 2>/dev/null
    sleep 2
    # 日志写到持久目录（/tmp 是内存盘，会被 systemd 清掉，重启也没了）
    mkdir -p /home/root/run_logs
    if [ -s "$LOG" ]; then
        mv -f "$LOG" "/home/root/run_logs/run_$(date +%Y%m%d_%H%M%S).log" 2>/dev/null
    fi
    # 只保留最近 20 份，避免占满 rootfs
    ls -t /home/root/run_logs/run_*.log 2>/dev/null | tail -n +21 | while read f; do rm -f "$f"; done
    setsid sh -c "sleep 86400 | stdbuf -oL ./beaconRace $MODE > $LOG 2>&1" < /dev/null > /dev/null 2>&1 &
    sleep 6
    echo "=== PROC ==="
    ps -A 2>/dev/null | grep beaconRace | grep -v grep || echo "  (not running)"
    echo "=== PORT ==="
    netstat -ltn 2>/dev/null | grep 9999 || echo "  9999 not listening"
    echo "=== CAMERA HOLDERS ==="
    holders
    echo "=== LOG ($LOG) ==="
    tail -8 "$LOG"
    ;;
  stop)
    pkill -x beaconRace 2>/dev/null && echo "beaconRace stopped (TERM)" || echo "beaconRace not running"
    sleep 1
    pkill -9 -x beaconRace 2>/dev/null     # 如果 TERM 没退干净再补刀
    pkill -f "sleep 86400" 2>/dev/null     # 清掉保持 stdin 的管道进程
    sleep 1
    echo "=== CAMERA HOLDERS ==="
    holders
    ;;
  cam)
    echo "=== 占用 /dev/video* 的进程 ==="
    holders
    if [ "${2:-}" = "--force" ]; then
      echo "=== 强制释放 ==="
      for pid in $(pids_holding_cam); do
        echo "  kill -TERM $pid"
        kill -TERM "$pid" 2>/dev/null
      done
      sleep 2
      for pid in $(pids_holding_cam); do
        echo "  kill -9 $pid (仍未释放)"
        kill -9 "$pid" 2>/dev/null
      done
      sleep 1
      echo "=== 释放后复查 ==="
      holders
    fi
    ;;
  status)
    echo "=== PROC ==="
    ps -A 2>/dev/null | grep beaconRace | grep -v grep || echo "  (not running)"
    echo "=== PORT ==="
    netstat -ltn 2>/dev/null | grep 9999 || echo "  9999 not listening"
    echo "=== CAMERA HOLDERS ==="
    holders
    ;;
  *)
    echo "usage: $0 start [--drive] | stop | status | cam [--force]"
    ;;
esac
