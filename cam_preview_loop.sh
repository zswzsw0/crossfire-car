#!/bin/sh
# 预览循环：ffmpeg 的 -listen 1 一次只服务一个客户端，客户端断开就退出，
# 所以这里套一层 while 让新连接随时能接上。由 cam_preview.sh start 拉起。
FF=/home/root/ffmpeg/bin/ffmpeg
LOG=/tmp/cam_preview.log
PORT=9999

export LD_LIBRARY_PATH=/home/root/ffmpeg/lib:$LD_LIBRARY_PATH

while true; do
    $FF -hide_banner -loglevel warning \
        -f v4l2 -input_format mjpeg -video_size 320x240 -framerate 30 \
        -i /dev/video0 -c:v copy -f mjpeg \
        -listen 1 tcp://0.0.0.0:$PORT >> $LOG 2>&1
    echo "--- ffmpeg exited, retry in 1s ---" >> $LOG
    sleep 1
done
