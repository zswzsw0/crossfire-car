#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证"局部对比度"能否把黑点和地面分开。

原理：黑点 = 比周围 15x15 区域的平均亮度明显更低的小块；
      地面大片均匀，局部均值与自身接近，不会被判为黑。
做法：V 通道 -> 15x15 均值(积分图) -> V < mean - C -> 后续尺寸/圆度过滤
用法: python3 analyze_adaptive.py frame.png [blockSize] [C]
"""
import sys

import gi

gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GdkPixbuf

PATH = sys.argv[1]
BLOCK = int(sys.argv[2]) if len(sys.argv) > 2 else 15
C = int(sys.argv[3]) if len(sys.argv) > 3 else 18

ROI_TOP = 0.45
AREA_MIN, AREA_MAX = 10, 400
MIN_W, MIN_H = 4, 3
ASP_MIN, ASP_MAX = 0.35, 2.2
FILL_MIN = 0.55

pb = GdkPixbuf.Pixbuf.new_from_file(PATH)
W, H, CH = pb.get_width(), pb.get_height(), pb.get_n_channels()
STRIDE = pb.get_rowstride()
PIX = pb.get_pixels()
roi_y = int(H * ROI_TOP)

V = [[0] * W for _ in range(H)]
for y in range(H):
    base = y * STRIDE
    for x in range(W):
        i = base + x * CH
        V[y][x] = max(PIX[i], PIX[i + 1], PIX[i + 2])

# 积分图
S = [[0] * (W + 1) for _ in range(H + 1)]
for y in range(H):
    rs = 0
    for x in range(W):
        rs += V[y][x]
        S[y + 1][x + 1] = S[y][x + 1] + rs

r = BLOCK // 2
mask = [[False] * W for _ in range(H)]
for y in range(roi_y, H):
    y0, y1 = max(0, y - r), min(H - 1, y + r)
    for x in range(W):
        x0, x1 = max(0, x - r), min(W - 1, x + r)
        area_n = (x1 - x0 + 1) * (y1 - y0 + 1)
        sm = S[y1 + 1][x1 + 1] - S[y0][x1 + 1] - S[y1 + 1][x0] + S[y0][x0]
        if V[y][x] * area_n < sm - C * area_n:
            mask[y][x] = True

n = sum(1 for y in range(roi_y, H) for x in range(W) if mask[y][x])
print(f"frame {W}x{H}  roi>={roi_y}  block={BLOCK} C={C}")
print(f"1) 局部对比掩膜: {n} 像素 (占 ROI {100.0*n/((H-roi_y)*W):.2f}%)")


def erode(m):
    out = [[False] * W for _ in range(H)]
    for y in range(roi_y, H):
        for x in range(W):
            if not m[y][x]:
                continue
            ok = True
            for dy in (-1, 0, 1):
                yy = y + dy
                if yy < roi_y or yy >= H:
                    ok = False
                    break
                for dx in (-1, 0, 1):
                    xx = x + dx
                    if xx < 0 or xx >= W or not m[yy][xx]:
                        ok = False
                        break
                if not ok:
                    break
            out[y][x] = ok
    return out


def dilate(m):
    out = [[False] * W for _ in range(H)]
    for y in range(roi_y, H):
        for x in range(W):
            if m[y][x]:
                out[y][x] = True
                continue
            hit = False
            for dy in (-1, 0, 1):
                yy = y + dy
                if yy < roi_y or yy >= H:
                    continue
                for dx in (-1, 0, 1):
                    xx = x + dx
                    if 0 <= xx < W and m[yy][xx]:
                        hit = True
                        break
                if hit:
                    break
            out[y][x] = hit
    return out


opened = dilate(erode(mask))
closed = erode(dilate(opened))
print(f"2) 3x3 开+闭后: {sum(1 for y in range(roi_y,H) for x in range(W) if closed[y][x])}")

seen = [[False] * W for _ in range(H)]
res = []
for y in range(roi_y, H):
    for x in range(W):
        if not closed[y][x] or seen[y][x]:
            continue
        st = [(x, y)]
        seen[y][x] = True
        area = 0
        x0 = x1 = x
        y0 = y1 = y
        while st:
            cx, cy = st.pop()
            area += 1
            if cx < x0: x0 = cx
            if cx > x1: x1 = cx
            if cy < y0: y0 = cy
            if cy > y1: y1 = cy
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < W and roi_y <= ny < H and closed[ny][nx] and not seen[ny][nx]:
                    seen[ny][nx] = True
                    st.append((nx, ny))
        w = x1 - x0 + 1
        h = y1 - y0 + 1
        asp = h / float(w)
        fill = area / float(w * h)
        ok = (AREA_MIN <= area <= AREA_MAX and w >= MIN_W and h >= MIN_H
              and ASP_MIN <= asp <= ASP_MAX and fill >= FILL_MIN)
        res.append((area, w, h, asp, fill, (x0 + x1) // 2, (y0 + y1) // 2, ok))

good = [t for t in res if t[7]]
print(f"3) 连通域 {len(res)}，通过全部过滤: {len(good)}  <-- blk")
for area, w, h, asp, fill, cx, cy, _ in sorted(good, key=lambda t: -t[0])[:8]:
    print(f"     通过: area={area:4d} {w:3d}x{h:3d} asp={asp:.2f} fill={fill:.2f} cx={cx} cy={cy}")
bad = [t for t in res if not t[7] and t[0] >= AREA_MIN]
if bad:
    print("   未通过的大块（前 5）:")
    for area, w, h, asp, fill, cx, cy, _ in sorted(bad, key=lambda t: -t[0])[:5]:
        print(f"     area={area:4d} {w:3d}x{h:3d} asp={asp:.2f} fill={fill:.2f} cy={cy}")
