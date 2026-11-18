#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 320x240 画面压成 ASCII 灰度图，便于在终端里"看"图像。
越暗字符越靠前：@ = 最暗，空格 = 最亮。
用法: python3 analyze_ascii.py frame.png [宽] [高]
"""
import sys

import gi

gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GdkPixbuf

PATH = sys.argv[1]
COLS = int(sys.argv[2]) if len(sys.argv) > 2 else 96
ROWS = int(sys.argv[3]) if len(sys.argv) > 3 else 40

RAMP = "@%#*+=-:. "   # 暗 -> 亮

pb = GdkPixbuf.Pixbuf.new_from_file(PATH)
W, H, CH = pb.get_width(), pb.get_height()
STRIDE = pb.get_rowstride()
PIX = pb.get_pixels()

roi_y = int(H * 0.45)
print(f"frame {W}x{H}   ROI 从第 {roi_y} 行开始（下面标记 ==== ROI 线 ====）")

for ry in range(ROWS):
    y0 = int(ry * H / ROWS)
    y1 = max(y0 + 1, int((ry + 1) * H / ROWS))
    line = []
    for rx in range(COLS):
        x0 = int(rx * W / COLS)
        x1 = max(x0 + 1, int((rx + 1) * W / COLS))
        tot = 0
        n = 0
        for y in range(y0, y1):
            base = y * STRIDE
            for x in range(x0, x1):
                i = base + x * CH
                tot += max(PIX[i], PIX[i + 1], PIX[i + 2])
                n += 1
        v = tot // max(n, 1)
        idx = min(len(RAMP) - 1, v * len(RAMP) // 256)
        line.append(RAMP[idx])
    prefix = "ROI " if y0 < roi_y <= y1 else "    "
    print(f"{y0:3d} {prefix}" + "".join(line))
