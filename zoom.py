"""放大打印画面中央区域，看黑点到底长什么样。
用法: python zoom.py <png> x0 x1 y0 y1 [step]
"""
import sys

from PIL import Image

PATH = sys.argv[1]
x0, x1 = int(sys.argv[2]), int(sys.argv[3])
y0, y1 = int(sys.argv[4]), int(sys.argv[5])
STEP = int(sys.argv[6]) if len(sys.argv) > 6 else 2

im = Image.open(PATH).convert("RGB")
px = im.load()
print(f"zoom x[{x0},{x1}) y[{y0},{y1}) step={STEP}")
print("      " + "".join(str((x // 10) % 10) if x % 10 == 0 else " " for x in range(x0, x1, STEP)))
for y in range(y0, y1, STEP):
    line = []
    for x in range(x0, x1, STEP):
        V = max(px[x, y])
        if V <= 120:
            line.append("@")
        elif V <= 150:
            line.append("#")
        elif V <= 165:
            line.append("+")
        elif V <= 180:
            line.append("-")
        else:
            line.append(".")
    print(f"y{y:3d} " + "".join(line))
