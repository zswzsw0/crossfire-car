#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""黑点阈值扫描：找出黑点的真实亮度档位，并模拟 5x5 形态学开运算的影响。

用法: python3 analyze_black2.py /path/frame.png
"""
import sys
from collections import Counter

import gi

gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GdkPixbuf

PATH = sys.argv[1]
ROI_TOP = 0.45
AREA_MIN = 20

pb = GdkPixbuf.Pixbuf.new_from_file(PATH)
W, H, CH = pb.get_width(), pb.get_height(), pb.get_n_channels()
STRIDE = pb.get_rowstride()
PIX = pb.get_pixels()
roi_y = int(H * ROI_TOP)
print(f"frame {W}x{H}  roi 从第 {roi_y} 行往下")

# 预先算好每个像素的 V / S
V = [[0] * W for _ in range(H)]
S = [[0] * W for _ in range(H)]
for y in range(H):
    base = y * STRIDE
    for x in range(W):
        i = base + x * CH
        r, g, b = PIX[i], PIX[i + 1], PIX[i + 2]
        mx = max(r, g, b)
        mn = min(r, g, b)
        V[y][x] = mx
        S[y][x] = 0 if mx == 0 else int(255.0 * (mx - mn) / mx)

roi_cnt = (H - roi_y) * W
for vt in (60, 90, 120, 150, 180):
    n = sum(1 for y in range(roi_y, H) for x in range(W) if V[y][x] <= vt)
    print(f"  V<={vt:3d}: ROI 内像素占比 {100.0*n/roi_cnt:5.2f}%")


def components(mask):
    seen = [[False] * W for _ in range(H)]
    out = []
    for y in range(roi_y, H):
        for x in range(W):
            if not mask[y][x] or seen[y][x]:
                continue
            stack = [(x, y)]
            seen[y][x] = True
            area = 0
            x0 = x1 = x
            y0 = y1 = y
            while stack:
                cx, cy = stack.pop()
                area += 1
                if cx < x0: x0 = cx
                if cx > x1: x1 = cx
                if cy < y0: y0 = cy
                if cy > y1: y1 = cy
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < W and roi_y <= ny < H and mask[ny][nx] and not seen[ny][nx]:
                        seen[ny][nx] = True
                        stack.append((nx, ny))
            out.append((area, x1 - x0 + 1, y1 - y0 + 1))
    return out


def erode5(mask):
    """5x5 腐蚀（开运算的第一步）：窗口内全为 1 才保留。用积分图加速。"""
    integral = [[0] * (W + 1) for _ in range(H + 1)]
    for y in range(H):
        row_sum = 0
        for x in range(W):
            row_sum += 1 if mask[y][x] else 0
            integral[y + 1][x + 1] = integral[y][x + 1] + row_sum
    out = [[False] * W for _ in range(H)]
    r = 2
    for y in range(roi_y, H):
        for x in range(W):
            x0 = max(0, x - r); x1 = min(W - 1, x + r)
            y0 = max(roi_y, y - r); y1 = min(H - 1, y + r)
            total = (integral[y1 + 1][x1 + 1] - integral[y0][x1 + 1]
                     - integral[y1 + 1][x0] + integral[y0][x0])
            if total == (x1 - x0 + 1) * (y1 - y0 + 1):
                out[y][x] = True
    return out


print("\n=== 阈值扫描（S<=110）: 原始候选 / 经过 5x5 开运算后候选 ===")
for vt in (90, 120, 150, 180):
    mask = [[False] * W for _ in range(H)]
    for y in range(roi_y, H):
        for x in range(W):
            mask[y][x] = (V[y][x] <= vt and S[y][x] <= 110)
    raw = [c for c in components(mask) if c[0] >= AREA_MIN]
    er = erode5(mask)
    # 开运算 = 腐蚀 + 膨胀，这里直接用腐蚀后的掩膜做连通域，近似看"活下来多少"
    eroded = [c for c in components(er) if c[0] >= AREA_MIN]
    print(f"  V<={vt:3d}: 原始 {len(raw):3d} 个   开运算后 {len(eroded):3d} 个", end="")
    if raw:
        top = sorted(raw, reverse=True)[:3]
        print("   原始最大的: " + ", ".join(f"{a}px({w}x{h})" for a, w, h in top))
    else:
        print()
