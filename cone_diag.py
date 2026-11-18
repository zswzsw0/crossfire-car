"""列出画面里的红/蓝锥候选，并判断是否落在"挡路走廊"里。
用法: python cone_diag.py <png>
HSV 范围取自 hsv_saved.txt:
  HUE_RED_1 = (0,80,60, 10,255,255)
  HUE_RED_2 = (156,80,60, 180,255,255)
  HUE_BLUE  = (90,60,40, 130,255,255)
走廊: |cx - ref| <= 45 + 60*(cy/240)，ref = 黑点线的 mid_x（这里用画面中线 160 近似）
"""
import sys
from PIL import Image

PATH = sys.argv[1]
IMG_W, IMG_H = 320, 240
ROI_TOP = 0.45
CONE_MIN_ASPECT, CONE_MAX_ASPECT = 0.70, 2.5
AREA_MIN = 40
CONE_PATH_HALF_W = 45.0

im = Image.open(PATH).convert("RGB")
W, H = im.size
px = im.load()
roi = int(H * ROI_TOP)


def hsv(r, g, b):
    mx, mn = max(r, g, b), min(r, g, b)
    v = mx
    s = 0 if mx == 0 else int(255 * (mx - mn) / mx)
    if mx == mn:
        hh = 0
    elif mx == r:
        hh = int(30 * ((g - b) / float(mx - mn))) % 180
    elif mx == g:
        hh = int(30 * (2 + (b - r) / float(mx - mn)))
    else:
        hh = int(30 * (4 + (r - g) / float(mx - mn)))
    return hh, s, v


def in_red(hh, s, v):
    return s >= 80 and v >= 60 and ((0 <= hh <= 10) or (156 <= hh <= 180))


def in_blue(hh, s, v):
    return 90 <= hh <= 130 and s >= 60 and v >= 40


for name, pred in (("RED", in_red), ("BLUE", in_blue)):
    mask = [[False] * W for _ in range(H)]
    for y in range(roi, H):
        for x in range(W):
            if pred(*hsv(*px[x, y])):
                mask[y][x] = True
    seen = [[False] * W for _ in range(H)]
    print(f"\n=== {name} ===")
    for y in range(roi, H):
        for x in range(W):
            if not mask[y][x] or seen[y][x]:
                continue
            st = [(x, y)]
            seen[y][x] = True
            a = 0
            x0 = x1 = x
            y0 = y1 = y
            while st:
                cx, cy = st.pop()
                a += 1
                x0, x1 = min(x0, cx), max(x1, cx)
                y0, y1 = min(y0, cy), max(y1, cy)
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < W and roi <= ny < H and mask[ny][nx] and not seen[ny][nx]:
                        seen[ny][nx] = True
                        st.append((nx, ny))
            w, h = x1 - x0 + 1, y1 - y0 + 1
            asp = h / float(w)
            if a < AREA_MIN:
                continue
            ccx, ccy = (x0 + x1) // 2, (y0 + y1) // 2
            thr = CONE_PATH_HALF_W + 60.0 * (ccy / float(IMG_H))
            in_path = abs(ccx - IMG_W / 2) <= thr
            print(f"  area={a:5d} {w:3d}x{h:3d} asp={asp:4.2f} cx={ccx:3d} cy={ccy:3d} "
                  f"corridor={'YES' if in_path else 'no ':3s} (thr={thr:.0f}) aspect_ok={CONE_MIN_ASPECT<=asp<=CONE_MAX_ASPECT}")
