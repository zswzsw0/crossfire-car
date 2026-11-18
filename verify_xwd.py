#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统计一张 xwd 截图的像素情况，用来判断窗口里是不是真有画面（而不是纯色/黑屏）。
用法: python3 verify_xwd.py /tmp/win.xwd
"""
import struct
import sys
from collections import Counter

path = sys.argv[1]
data = open(path, "rb").read()

# XWD 头是 25 个大端 32 位整数，按规范位置取值
head = struct.unpack(">25I", data[:100])
hsize = head[0]      # 头部字节数
depth = head[3]      # 位深
w, h = head[4], head[5]
bpp = head[11]       # 每像素位数
bpl = head[12]       # 每行字节数
ncolors = head[19]
winw, winh = head[20], head[21]

off = hsize + ncolors * 12
pixels = data[off:off + bpl * h]

nbytes = bpp // 8
vals = []
colors = Counter()
step = max(1, bpl // 160)  # 每行取约 160 个点
for y in range(0, h, 3):
    row = pixels[y * bpl:(y + 1) * bpl]
    for x in range(0, w, max(1, step // nbytes)):
        i = x * nbytes
        if i + 3 > len(row):
            continue
        rgb = (row[i], row[i + 1], row[i + 2])
        colors[rgb] += 1
        vals.append(sum(rgb) / 3.0)

if not vals:
    print("NO_PIXELS")
    sys.exit(1)

mean = sum(vals) / len(vals)
var = sum((v - mean) ** 2 for v in vals) / len(vals)
print(f"尺寸 {winw}x{winh}  采样 {len(vals)} 点  平均亮度 {mean:.1f}  标准差 {var ** 0.5:.1f}")
print(f"不同颜色数 {len(colors)}  最常见 3 种: {colors.most_common(3)}")
print("判定:", "有画面内容" if var ** 0.5 > 5 and len(colors) > 20 else "可能是纯色/黑屏")
