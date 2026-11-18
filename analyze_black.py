#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用与 beaconRace 相同的 HSV / 几何规则统计一帧里的黑点候选。
依赖 python3-gi（GdkPixbuf 解码 PNG/JPEG），不需要 numpy / opencv。

用法: python3 analyze_black.py /path/frame.png
"""
import sys
from collections import Counter

import gi

gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GdkPixbuf

PATH = sys.argv[1]
ROI_TOP = 0.45          # roi_y_top
S_MAX = 80              # HUE_BLACK 的 S 上限
AREA_MIN, AREA_MAX = 20, 700      # cfg 当前值
MIN_W, MIN_H = 8, 6               # 源码常量
ASP_MIN, ASP_MAX = 0.20, 2.2

pb = GdkPixbuf.Pixbuf.new_from_file(PATH)
W, H, CH = pb.get_width(), pb.get_height(), pb.get_n_channels()
STRIDE = pb.get_rowstride()
PIX = pb.get_pixels()
print(f"frame {W}x{H} channels={CH} roi_top={int(H*ROI_TOP)}")


def components(vmax):
    """返回连通域列表 (area, w, h, cx, cy)。"""
    mask = bytearray(W * H)
    for y in range(int(H * ROI_TOP), H):
        base = y * STRIDE
        row = y * W
        for x in range(W):
            i = base + x * CH
            r, g, b = PIX[i], PIX[i + 1], PIX[i + 2]
            mx = max(r, g, b)
            if mx == 0:
                v, s = 0, 0
            else:
                v = mx
                s = 255.0 * (mx - min(r, g, b)) / mx
            if v <= vmax and s <= S_MAX:
                mask[row + x] = 1

    seen = bytearray(W * H)
    out = []
    for idx in range(W * H):
        if not mask[idx] or seen[idx]:
            continue
        stack = [idx]
        seen[idx] = 1
        area = 0
        minx = maxx = idx % W
        miny = maxy = idx // W
        while stack:
            p = stack.pop()
            area += 1
            px, py = p % W, p // W
            if px < minx: minx = px
            if px > maxx: maxx = px
            if py < miny: miny = py
            if py > maxy: maxy = py
            for d, ok in ((-1, px > 0), (1, px < W - 1), (-W, py > 0), (W, py < H - 1)):
                q = p + d
                if ok and mask[q] and not seen[q]:
                    seen[q] = 1
                    stack.append(q)
        w = maxx - minx + 1
        h = maxy - miny + 1
        out.append((area, w, h, (minx + maxx) / 2.0, (miny + maxy) / 2.0))
    return out


for vmax in (60, 75, 90, 110):
    comps = components(vmax)
    cands = [c for c in comps if c[0] >= AREA_MIN]
    passed = []
    for area, w, h, cx, cy in cands:
        asp = h / float(w)
        if AREA_MIN <= area <= AREA_MAX and w >= MIN_W and h >= MIN_H and ASP_MIN <= asp <= ASP_MAX:
            passed.append((area, w, h, round(asp, 2)))
    print(f"\n=== V<={vmax} S<={S_MAX} ===")
    print(f"  连通域(area>={AREA_MIN}): {len(cands)}   通过全部过滤: {len(passed)}")
    if cands:
        bucket = Counter()
        for area, *_ in cands:
            if area < 50: bucket["20-49"] += 1
            elif area < 150: bucket["50-149"] += 1
            elif area < 400: bucket["150-399"] += 1
            elif area < AREA_MAX: bucket[f"400-{AREA_MAX-1}"] += 1
            elif area < 1500: bucket[f"{AREA_MAX}-1499"] += 1
            else: bucket[">=1500"] += 1
        print("  面积分布:", dict(sorted(bucket.items())))
        big = [c for c in cands if c[0] > AREA_MAX]
        print(f"  因 area>{AREA_MAX} 被丢: {len(big)} 个", end="")
        if big:
            print("  最大的: " + ", ".join(f"area={a} {w}x{h}" for a, w, h, _, _ in sorted(big, reverse=True)[:4]))
        else:
            print()
        if passed:
            print("  通过样例:", passed[:5])
