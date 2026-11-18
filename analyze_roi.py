#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只画 ROI 区域，按亮度分级用字符标出，用来判断黑点在哪、边缘暗角多强。
   @ = V<=150   # = 151-160   + = 161-170   . = 171-180   (空格) = >180
用法: python3 analyze_roi.py frame.png [宽] [高]
"""
import sys

import gi

gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GdkPixbuf

PATH = sys.argv[1]
COLS = int(sys.argv[2]) if len(sys.argv) > 2 else 120
ROWS = int(sys.argv[3]) if len(sys.argv) > 3 else 30

pb = GdkPixbuf.Pixbuf.new_from_file(PATH)
W, H, CH = pb.get_width(), pb.get_height(), pb.get_n_channels()
STRIDE = pb.get_rowstride()
PIX = pb.get_pixels()
roi_y = int(H * 0.45)
rh = H - roi_y

print(f"ROI: y {roi_y}..{H}  ({COLS}x{ROWS} 字符, 每字符约 {W/COLS:.1f}x{rh/ROWS:.1f} 像素)")
print("     " + "".join(str((i // 10) % 10) if i % 10 == 0 else " " for i in range(COLS)))
for ry in range(ROWS):
    y0 = roi_y + int(ry * rh / ROWS)
    y1 = roi_y + max(1, int((ry + 1) * rh / ROWS))
    line = []
    for rx in range(COLS):
        x0 = int(rx * W / COLS)
        x1 = max(x0 + 1, int((rx + 1) * W / COLS))
        mn = 999
        for y in range(y0, y1):
            base = y * STRIDE
            for x in range(x0, x1):
                i = base + x * CH
                mn = min(mn, max(PIX[i], PIX[i + 1], PIX[i + 2]))
        if mn <= 150:
            line.append("@")
        elif mn <= 160:
            line.append("#")
        elif mn <= 170:
            line.append("+")
        elif mn <= 180:
            line.append(".")
        else:
            line.append(" ")
    print(f"y{y0:3d} " + "".join(line))
