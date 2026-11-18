"""在 PC 上解码视频并分析黑点：ASCII 预览 + 候选统计。
依赖: imageio-ffmpeg (自带 ffmpeg), Pillow
用法: python anal_video.py <video> [fps] [每帧都打印ASCII=1]
"""
import os
import subprocess
import sys
import tempfile

from PIL import Image
import imageio_ffmpeg

VIDEO = sys.argv[1]
FPS = sys.argv[2] if len(sys.argv) > 2 else "2"
SHOW = (len(sys.argv) > 3 and sys.argv[3] == "1")

W, H = 320, 240
ROI_TOP = 0.45
RAMP = "@%#*+=-:. "
BLOCK, C = 15, 18
AREA_MIN, AREA_MAX = 10, 400
MIN_W, MIN_H = 4, 3
ASP_MIN, ASP_MAX = 0.35, 2.2
FILL_MIN = 0.55

tmp = tempfile.mkdtemp(prefix="vf_")
exe = imageio_ffmpeg.get_ffmpeg_exe()
pattern = os.path.join(tmp, "f_%02d.png")
cmd = [exe, "-hide_banner", "-loglevel", "error", "-i", VIDEO,
       "-vf", f"fps={FPS},scale={W}:{H}", "-y", pattern]
subprocess.run(cmd, check=True)
frames = sorted(f for f in os.listdir(tmp) if f.endswith(".png"))
print(f"视频已解码: {len(frames)} 帧 (fps={FPS}, {W}x{H})")


def stats(im):
    px = im.load()
    V = [[max(px[x, y][:3]) for x in range(W)] for y in range(H)]
    # 积分图做 15x15 局部均值
    S = [[0] * (W + 1) for _ in range(H + 1)]
    for y in range(H):
        rs = 0
        for x in range(W):
            rs += V[y][x]
            S[y + 1][x + 1] = S[y][x + 1] + rs
    r = BLOCK // 2
    roi = int(H * ROI_TOP)
    mask = [[False] * W for _ in range(H)]
    for y in range(roi, H):
        y0, y1 = max(0, y - r), min(H - 1, y + r)
        for x in range(W):
            x0, x1 = max(0, x - r), min(W - 1, x + r)
            n = (x1 - x0 + 1) * (y1 - y0 + 1)
            s = S[y1 + 1][x1 + 1] - S[y0][x1 + 1] - S[y1 + 1][x0] + S[y0][x0]
            if V[y][x] * n < s - C * n:
                mask[y][x] = True
    # 3x3 开
    op = [[False] * W for _ in range(H)]
    for y in range(roi, H):
        for x in range(W):
            if not mask[y][x]:
                continue
            ok = all(0 <= x + dx < W and roi <= y + dy < H and mask[y + dy][x + dx]
                     for dy in (-1, 0, 1) for dx in (-1, 0, 1))
            op[y][x] = ok
    # 连通域
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
            asp = h / float(w)
            fill = a / float(w * h)
            ok = (AREA_MIN <= a <= AREA_MAX and w >= MIN_W and h >= MIN_H
                  and ASP_MIN <= asp <= ASP_MAX and fill >= FILL_MIN)
            cands.append((a, w, h, asp, fill, (x0 + x1) // 2, (y0 + y1) // 2, ok))
    return V, cands, roi


def ascii_map(V, roi, cols=96, rows=28):
    out = []
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
        out.append(f"{y0:3d}{tag}|" + "".join(line))
    return "\n".join(out)


for idx, name in enumerate(frames):
    im = Image.open(os.path.join(tmp, name)).convert("RGB")
    V, cands, roi = stats(im)
    good = [c for c in cands if c[7]]
    print(f"\n===== {name}  候选={len(cands)} 通过={len(good)} =====")
    if SHOW or idx == 0:
        print("    " + "".join(str((i // 10) % 10) if i % 10 == 0 else " " for i in range(96)))
        print(ascii_map(V, roi))
    for a, w, h, asp, fill, cx, cy, _ in sorted(good, key=lambda t: -t[0])[:6]:
        print(f"   ✔ area={a:4d} {w:3d}x{h:3d} asp={asp:.2f} fill={fill:.2f} cx={cx} cy={cy}")
    bad = [c for c in cands if not c[7]]
    if bad:
        a, w, h, asp, fill, cx, cy, _ = max(bad, key=lambda t: t[0])
        print(f"   ✘ 最大的未通过: area={a} {w}x{h} asp={asp:.2f} fill={fill:.2f} cy={cy}")
