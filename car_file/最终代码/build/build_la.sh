#!/bin/sh
# 在虚拟机上交叉编译 beaconRace（不依赖 cmake / make，直接用工具链里的 g++）
# 用法:
#   sh build_la.sh                 # 输出 build_la/beaconRace
#   OPENCV_ROOT=/home/mark/opencv sh build_la.sh
set -e

TC=/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6
CXX=$TC/bin/loongarch64-linux-gnu-g++
READELF=$TC/bin/loongarch64-linux-gnu-readelf

SRC_DIR=$(cd "$(dirname "$0")" && pwd)
CV=${OPENCV_ROOT:-/home/mark/opencv}
OUT=${1:-build_la/beaconRace}

if [ ! -x "$CXX" ]; then echo "找不到交叉编译器: $CXX"; exit 1; fi
if [ ! -d "$CV/include/opencv4" ]; then echo "找不到 OpenCV 头文件: $CV/include/opencv4"; exit 1; fi

mkdir -p "$SRC_DIR/build_la"

echo "== 编译 =="
echo "   源码: $SRC_DIR"
echo "   OpenCV: $CV"
$CXX -std=c++17 -O3 -pthread -Wall \
    -I "$SRC_DIR/includes" \
    -I "$CV/include/opencv4" \
    "$SRC_DIR"/src/*.cpp \
    "$SRC_DIR"/compat/cv_compat_411.cpp \
    "$SRC_DIR"/beaconRace/beaconRace.cpp \
    -L "$CV/lib" \
    -Wl,-rpath,/home/root/opencv/lib \
    -Wl,-rpath,/home/mark/opencv/lib \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio \
    -lopencv_highgui -lopencv_calib3d -lopencv_features2d -lopencv_flann \
    -o "$SRC_DIR/$OUT"

echo "== 产物 =="
ls -l "$SRC_DIR/$OUT"
echo "== 架构确认 =="
$READELF -h "$SRC_DIR/$OUT" | grep -E 'Class:|Machine:'
echo "== 依赖的动态库 =="
$READELF -d "$SRC_DIR/$OUT" | grep -E 'NEEDED|RUNPATH|RPATH'
