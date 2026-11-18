#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""持续采样上位机窗口画面，输出时间线，用于无人值守地"看"画面变化。

不占用视频流连接（走 X 截图），只依赖 xwd + python3。
每行输出: 时间  红%  蓝%  绿%  亮度  与上一帧差异%

用法:
    DISPLAY=:0 XAUTHORITY=$HOME/.Xauthority python3 watch_scene.py [间隔秒] [总时长秒]
"""
import os
import struct
import subprocess
import sys
import time

INTERVAL = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 900.0
OUT = "/tmp/scene_watch.log"
SHOT = "/tmp/.scene_shot.xwd"


def window_id():
    """每次重新查窗口 id，viewer 重启后窗口 id 会变。"""
    try:
        out = subprocess.check_output(["xwininfo", "-root", "-tree"],
                                      stderr=subprocess.DEVNULL, timeout=10).decode("utf-8", "replace")
    except Exception:
        return None
    for line in out.splitlines():
        if "beaconRace" in line and "gtk_viewer" in line:
            return line.split()[0]
    return None


def grab(wid):
    try:
        subprocess.run(["xwd", "-id", wid, "-out", SHOT],
                       stderr=subprocess.DEVNULL, timeout=10, check=False)
        data = open(SHOT, "rb").read()
    except Exception:
        return None
    if len(data) < 100:
        return None
    return data


def stats(data):
    try:
        head = struct.unpack(">25I", data[:100])
    except struct.error:
        return None
    hsize, w, h, bpp, bpl, ncolors = head[0], head[4], head[5], head[11], head[12], head[19]
    off = hsize + ncolors * 12
    pix = data[off:off + bpl * h]
    if not pix:
        return None
    nbytes = bpp // 8
    if nbytes < 3:
        return None

    total = red = blue = green = 0
    lum = 0
    step = max(1, (w // 120) * nbytes)
    # 取样点也顺便留一份灰度用于帧间差异
    sample = []
    for y in range(0, h, 4):
        row = pix[y * bpl:(y + 1) * bpl]
        for x in range(0, w, step):
            i = x * nbytes
            if i + 3 > len(row):
                continue
            r, g, b = row[i], row[i + 1], row[i + 2]
            total += 1
            lum += (r + g + b) / 3.0
            sample.append((r + g + b) // 3)
            if r > 90 and r > 1.6 * g and r > 1.6 * b:
                red += 1
            elif b > 90 and b > 1.5 * r and b > 1.3 * g:
                blue += 1
            elif g > 90 and g > 1.4 * r and g > 1.2 * b:
                green += 1
    if total == 0:
        return None
    return {
        "w": w, "h": h,
        "red": 100.0 * red / total,
        "blue": 100.0 * blue / total,
        "green": 100.0 * green / total,
        "lum": lum / total,
        "sample": sample,
    }


def diff_pct(a, b):
    if not a or not b or len(a) != len(b):
        return -1.0
    changed = sum(1 for x, y in zip(a, b) if abs(x - y) > 12)
    return 100.0 * changed / len(a)


def main():
    log = open(OUT, "w", buffering=1)
    t0 = time.time()
    prev = None
    last_note = ""
    log.write(f"# 开始监看，间隔 {INTERVAL}s，总时长 {DURATION}s\n")
    while time.time() - t0 < DURATION:
        wid = window_id()
        if not wid:
            line = "窗口不存在（viewer 没开或已关）"
            if line != last_note:
                log.write(f"{time.strftime('%H:%M:%S')}  {line}\n")
                last_note = line
            time.sleep(INTERVAL)
            continue
        data = grab(wid)
        s = stats(data) if data else None
        if not s:
            time.sleep(INTERVAL)
            continue
        d = diff_pct(prev["sample"], s["sample"]) if prev else -1.0
        prev = s
        log.write("{ts}  红{red:6.2f}% 蓝{blue:6.2f}% 绿{green:6.2f}% 亮度{lum:6.1f} 变化{d:6.1f}%  ({w}x{h})\n".format(
            ts=time.strftime("%H:%M:%S"), d=d, **s))
        time.sleep(INTERVAL)
    log.write("# 监看结束\n")


if __name__ == "__main__":
    main()
