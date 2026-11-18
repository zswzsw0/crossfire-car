#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统计一张 xwd 截图里各颜色族占多少像素，用来判断色卡是否在视野内。
用法: python3 count_colors.py /tmp/win.xwd
"""
import struct
import sys

path = sys.argv[1]
data = open(path, "rb").read()
head = struct.unpack(">25I", data[:100])
hsize = head[0]
w, h = head[4], head[5]
bpp = head[11]
bpl = head[12]
ncolors = head[19]

off = hsize + ncolors * 12
pixels = data[off:off + bpl * h]
nbytes = bpp // 8

total = red = blue = green = yellow = dark = 0
step = max(1, w // 200 * nbytes)
for y in range(0, h, 2):
    row = pixels[y * bpl:(y + 1) * bpl]
    for x in range(0, w, step):
        i = x * nbytes
        if i + 3 > len(row):
            continue
        r, g, b = row[i], row[i + 1], row[i + 2]
        total += 1
        if r < 40 and g < 40 and b < 40:
            dark += 1
        elif r > 90 and r > 1.6 * g and r > 1.6 * b:
            red += 1
        elif b > 90 and b > 1.5 * r and b > 1.3 * g:
            blue += 1
        elif g > 90 and g > 1.4 * r and g > 1.2 * b:
            green += 1
        elif r > 120 and g > 120 and b < 100:
            yellow += 1

if total == 0:
    print("NO_PIXELS")
    sys.exit(1)

def pct(n):
    return 100.0 * n / total

print(f"采样 {total} 点  ({w}x{h})")
print(f"  红色系 {pct(red):5.2f}%   蓝色系 {pct(blue):5.2f}%")
print(f"  绿色系 {pct(green):5.2f}%   黄色系 {pct(yellow):5.2f}%   暗部 {pct(dark):5.2f}%")
print()
mx = max(red, blue)
if pct(mx) < 5:
    print("判定: 画面里没有大面积红/蓝色块 —— 色卡没进视野，或离得太远")
elif pct(mx) < 50:
    print(f"判定: 有色块但只占 {pct(mx):.1f}%，低于 cfg 的 card_fill_ratio=0.50 门槛")
else:
    print(f"判定: 色块占 {pct(mx):.1f}%，超过 0.50 门槛，应能触发识别")
