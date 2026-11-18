#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 beaconRace 当前参数逐级模拟黑点检测，定位"黑点在哪一步被筛掉"。

阶段: HSV 掩膜 -> ROI -> 3x3 开 -> 3x3 闭 -> 连通域(area>=10)
      -> 宽高 >=4x3 -> 长宽比 0.35~2.2 -> 圆度 fill>=0.42
用法: python3 analyze_black3.py /path/frame.png [Vmax] [Smax]
"""
import sys

import gi

gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GdkPixbuf

PATH = sys.argv[1]
VMAX = int(sys.argv[2]) if len(sys.argv) > 2 else 120
SMAX = int(sys.argv[3]) if len(sys.argv) > 3 else 110

ROI_TOP = 0.45
AREA_MIN, AREA_MAX = 10, 2500
MIN_W, MIN_H = 4, 3
ASP_MIN, ASP_MAX = 0.35, 2.2
FILL_MIN = 0.42

pb = GdkPixbuf.Pixbuf.new_from_file(PATH)
W, H, CH = pb.get_width(), pb.get_height(), pb.get_n_channels()
STRIDE = pb.get_rowstride()
PIX = pb.get_pixels()
roi_y = int(H * ROI_TOP)

mask = [[False] * W for _ in range(H)]
for y in range(roi_y, H):
    base = y * STRIDE
    for x in range(W):
        i = base + x * CH
        r, g, b = PIX[i], PIX[i + 1], PIX[i + 2]
        mx = max(r, g, b)
        mn = min(r, g, b)
        if mx == 0:
            continue
        s = 255.0 * (mx - mn) / mx
        if mx <= VMAX and s <= SMAX:
            mask[y][x] = True

n_mask = sum(1 for y in range(roi_y, H) for x in range(W) if mask[y][x])
print(f"frame {W}x{H}  roi>={roi_y}  V<={VMAX} S<={SMAX}")
print(f"1) HSV 掩膜像素: {n_mask}")


def erode(m, r=1):
    out = [[False] * W for _ in range(H)]
    for y in range(roi_y, H):
        for x in range(W):
            if not m[y][x]:
                continue
            ok = True
            for dy in range(-r, r + 1):
                yy = y + dy
                if yy < roi_y or yy >= H:
                    ok = False
                    break
                for dx in range(-r, r + 1):
                    xx = x + dx
                    if xx < 0 or xx >= W or not m[yy][xx]:
                        ok = False
                        break
                if not ok:
                    break
            out[y][x] = ok
    return out


def dilate(m, r=1):
    out = [[False] * W for _ in range(H)]
    for y in range(roi_y, H):
        for x in range(W):
            if m[y][x]:
                out[y][x] = True
                continue
            hit = False
            for dy in range(-r, r + 1):
                yy = y + dy
                if yy < roi_y or yy >= H:
                    continue
                for dx in range(-r, r + 1):
                    xx = x + dx
                    if 0 <= xx < W and m[yy][xx]:
                        hit = True
                        break
                if hit:
                    break
            out[y][x] = hit
    return out


opened = dilate(erode(mask))
print(f"2) 3x3 开运算后: {sum(1 for y in range(roi_y,H) for x in range(W) if opened[y][x])}")
closed = erode(dilate(opened))
n_closed = sum(1 for y in range(roi_y, H) for x in range(W) if closed[y][x])
print(f"3) 3x3 闭运算后: {n_closed}")

# 连通域
seen = [[False] * W for _ in range(H)]
comps = []
for y in range(roi_y, H):
    for x in range(W):
        if not closed[y][x] or seen[y][x]:
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
                if 0 <= nx < W and roi_y <= ny < H and closed[ny][nx] and not seen[ny][nx]:
                    seen[ny][nx] = True
                    stack.append((nx, ny))
        comps.append((area, x1 - x0 + 1, y1 - y0 + 1, (x0 + x1) / 2, (y0 + y1) / 2))

c_area = [c for c in comps if AREA_MIN <= c[0] <= AREA_MAX]
print(f"4) 连通域总数 {len(comps)}，面积在 [{AREA_MIN},{AREA_MAX}]: {len(c_area)}")

c_size = [c for c in c_area if c[1] >= MIN_W and c[2] >= MIN_H]
print(f"5) 过了最小宽高({MIN_W}x{MIN_H}): {len(c_size)}")

c_asp = [c for c in c_size if ASP_MIN <= c[2] / float(c[1]) <= ASP_MAX]
print(f"6) 过了长宽比: {len(c_asp)}")

c_fill = [c for c in c_asp if c[0] / float(c[1] * c[2]) >= FILL_MIN]
print(f"7) 过了圆度({FILL_MIN}): {len(c_fill)}  <-- 最终 blk")
for area, w, h, cx, cy in sorted(c_fill, key=lambda t: -t[0]):
    print(f"     通过: area={area:5.0f} {w:3.0f}x{h:3.0f} fill={area/float(w*h):.2f} "
          f"cy={cy:.0f} cx={cx:.0f}")

if c_asp and not c_fill:
    print("   被圆度筛掉的候选（area/w/h/asp/fill）:")
    for area, w, h, cx, cy in sorted(c_asp, reverse=True)[:6]:
        print(f"     area={area} {w}x{h} asp={h/float(w):.2f} fill={area/float(w*h):.2f} cy={cy:.0f}")
if c_size and not c_asp:
    print("   被长宽比筛掉的:")
    for area, w, h, cx, cy in sorted(c_size, reverse=True)[:6]:
        print(f"     area={area} {w}x{h} asp={h/float(w):.2f} cy={cy:.0f}")
if comps and not c_area:
    print("   最大的几个连通域（都不在面积区间）:")
    for area, w, h, cx, cy in sorted(comps, reverse=True)[:6]:
        print(f"     area={area} {w}x{h} cy={cy:.0f}")
