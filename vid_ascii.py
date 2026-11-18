"""解码视频并打印 ASCII 预览（保持原始比例，纯 ASCII 输出）。
用法: python vid_ascii.py <video> [帧号] [宽字符数]
"""
import os
import subprocess
import sys
import tempfile

from PIL import Image
import imageio_ffmpeg

VIDEO = sys.argv[1]
WHICH = int(sys.argv[2]) if len(sys.argv) > 2 else 1
COLS = int(sys.argv[3]) if len(sys.argv) > 3 else 70

RAMP = "@%#*+=-:. "
tmp = tempfile.mkdtemp(prefix="va_")
exe = imageio_ffmpeg.get_ffmpeg_exe()
subprocess.run([exe, "-hide_banner", "-loglevel", "error", "-i", VIDEO,
                "-vf", "fps=1", "-y", os.path.join(tmp, "f_%02d.png")], check=True)
files = sorted(f for f in os.listdir(tmp) if f.endswith(".png"))
print("frames:", len(files))
if not files:
    sys.exit(1)
name = files[min(WHICH, len(files)) - 1]
im = Image.open(os.path.join(tmp, name)).convert("RGB")
W, H = im.size
px = im.load()
print(f"{name}  size={W}x{H}")

rows = max(1, int(COLS * H / W / 2))  # 字符高宽比约 2:1
for ry in range(rows):
    y0 = int(ry * H / rows)
    y1 = max(y0 + 1, int((ry + 1) * H / rows))
    line = []
    for rx in range(COLS):
        x0 = int(rx * W / COLS)
        x1 = max(x0 + 1, int((rx + 1) * W / COLS))
        tot = n = 0
        for y in range(y0, y1):
            for x in range(x0, x1):
                r, g, b = px[x, y]
                tot += max(r, g, b)
                n += 1
        v = tot // max(n, 1)
        line.append(RAMP[min(len(RAMP) - 1, v * len(RAMP) // 256)])
    print(f"{y0:4d}|" + "".join(line))
