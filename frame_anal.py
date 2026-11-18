"""分析一帧 PNG：ASCII 预览 + 黑点候选（自适应局部对比），纯 PC 侧运行。
用法: python frame_anal.py <png> [blockSize] [C]
"""
import sys

from PIL import Image

PATH = sys.argv[1]
BLOCK = int(sys.argv[2]) if len(sys.argv) > 2 else 15
C = int(sys.argv[3]) if len(sys.argv) > 3 else 18

W, H = 0, 0
im = Image.open(PATH).convert("RGB")
W, H = im.size
px = im.load()
print(f"frame {W}x{H}  block={BLOCK} C={C}")

V = [[max(px[x, y]) for x in range(W)] for y in range(H)]
RAMP = "@%#*+=-:. "

# ASCII 预览（含 ROI 标记）
roi = int(H * 0.45)
cols = 72
rows = max(1, int(cols * H / W / 2))
print("     " + "".join(str((i // 10) % 10) if i % 10 == 0 else " " for i in range(cols)))
for ry in range(rows):
    y0 = int(ry * H / rows)
    y1 = max(y0 + 1, int((ry + 1) * H / rows))
    line = []
    for rx in range(cols):
        x0 = int(rx * W / cols)
        x1 = max(x0 + 1, int((rx + 1) * W / cols))
        tot = n = 0
        for y in range(y0, y1):
            for x in range(x0, x1):
                tot += V[y][x]
                n += 1
        v = tot // max(n, 1)
        line.append(RAMP[min(len(RAMP) - 1, v * len(RAMP) // 256)])
    tag = "ROI" if y0 < roi <= y1 else "   "
    print(f"{y0:4d}{tag}|" + "".join(line))

# 局部均值积分图
S = [[0] * (W + 1) for _ in range(H + 1)]
for y in range(H):
    rs = 0
    for x in range(W):
        rs += V[y][x]
        S[y + 1][x + 1] = S[y][x + 1] + rs

r = BLOCK // 2
mask = [[False] * W for _ in range(H)]
for y in range(roi, H):
    y0, y1 = max(0, y - r), min(H - 1, y + r)
    for x in range(W):
        x0, x1 = max(0, x - r), min(W - 1, x + r)
        n = (x1 - x0 + 1) * (y1 - y0 + 1)
        s = S[y1 + 1][x1 + 1] - S[y0][x1 + 1] - S[y1 + 1][x0] + S[y0][x0]
        if V[y][x] * n < s - C * n:
            mask[y][x] = True

op = [[False] * W for _ in range(H)]
for y in range(roi, H):
    for x in range(W):
        if not mask[y][x]:
            continue
        ok = all(0 <= x + dx < W and roi <= y + dy < H and mask[y + dy][x + dx]
                 for dy in (-1, 0, 1) for dx in (-1, 0, 1))
        op[y][x] = ok

seen = [[False] * W for _ in range(H)]
cands = []
for y in range(roi, H):
    for x in range(W):
        if not op[y][x] or seen[y][x]:
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
                if 0 <= nx < W and roi <= ny < H and op[ny][nx] and not seen[ny][nx]:
                    seen[ny][nx] = True
                    st.append((nx, ny))
        w, h = x1 - x0 + 1, y1 - y0 + 1
        cands.append((a, w, h, h / float(w), a / float(w * h), (x0 + x1) // 2, (y0 + y1) // 2))

print(f"ROI 内连通域(>=10px): {len([c for c in cands if c[0] >= 10])}")
for a, w, h, asp, fill, cx, cy in sorted(cands, key=lambda t: -t[0])[:12]:
    if a < 10:
        continue
    print(f"   area={a:4d} {w:3d}x{h:3d} asp={asp:.2f} fill={fill:.2f} cx={cx:3d} cy={cy:3d}")
