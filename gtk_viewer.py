#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
虚拟机端上位机画面（自动重连版）
只依赖 python3-gi（GTK3 + GdkPixbuf），不需要 numpy / opencv。
协议与板子 ImgStream 一致：4 字节大端长度头 + 一帧 JPEG。

特点：
  * 断线自动重连（板子重启 beaconRace、网线抖动、程序崩溃都不用管窗口）
  * 可以先开窗口再启动 beaconRace，起来后自动接上
  * 重连期间保留最后一帧，不至于白屏
  * 画面随窗口大小等比缩放
  * ESC 退出，F11 全屏切换

用法：
    DISPLAY=:0 XAUTHORITY=$HOME/.Xauthority python3 gtk_viewer.py [ip] [port] [宽] [高]
    默认 192.168.1.3 : 9999 : 1000x760
"""
import socket
import struct
import sys
import threading
import queue
import time

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gtk, Gdk, GdkPixbuf, GLib  # noqa: E402

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.3"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 9999
WIN_W = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
WIN_H = int(sys.argv[4]) if len(sys.argv) > 4 else 760

# 每秒一行画面统计，供外部（人或脚本）监看用
STATUS_LOG = "/tmp/viewer_status.log"

frames = queue.Queue(maxsize=2)
state = {
    "stop": False,
    "status": "connecting",   # connecting / connected / reconnecting / closed
    "detail": "",
    "count": 0,
    "last_bytes": 0,
    "t0": time.time(),
    "fps": 0.0,
    "area": (WIN_W, WIN_H),
    "connects": 0,
    "last_pixbuf": None,
    "prev_sample": None,
    "mode": "",
}

RETRY_SEC = 1.0


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def reader():
    """后台线程：收帧 + 断线重连，全程不碰 GTK。

    兼容两种流格式，连上后自动嗅探：
      * framed —— beaconRace 的 ImgStream：4 字节大端长度 + 一帧 JPEG
      * raw    —— ffmpeg 直接吐的裸 MJPEG（预览模式，beaconRace 没运行时用）
    """
    while not state["stop"]:
        sock = None
        try:
            state["status"] = "connecting" if state["connects"] == 0 else "reconnecting"
            sock = socket.create_connection((HOST, PORT), timeout=5)
            sock.settimeout(10)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            state["connects"] += 1
            state["status"] = "connected"
            state["detail"] = ""
            log(f"已连接 {HOST}:{PORT}（第 {state['connects']} 次）")

            buf = b""

            def pull(n=65536):
                chunk = sock.recv(n)
                if not chunk:
                    raise EOFError("对端关闭连接")
                return chunk

            # 先攒 2 字节做嗅探：JPEG 一定以 FFD8 开头
            while len(buf) < 2:
                buf += pull()
            if buf[:2] == b"\xff\xd8":
                state["mode"] = "raw"
                log("流格式: 裸 MJPEG（ffmpeg 预览模式）")
            else:
                state["mode"] = "framed"
                log("流格式: 长度头 + JPEG（beaconRace）")

            while not state["stop"]:
                if state["mode"] == "framed":
                    while len(buf) < 4:
                        buf += pull()
                    ln = struct.unpack(">I", buf[:4])[0]
                    if ln <= 0 or ln > 8 * 1024 * 1024:
                        raise ValueError(f"帧长度异常 {ln}")
                    buf = buf[4:]
                    while len(buf) < ln:
                        buf += pull()
                    jpg, buf = buf[:ln], buf[ln:]
                else:
                    # 裸 MJPEG：找 SOI(FFD8) ... EOI(FFD9)
                    start = buf.find(b"\xff\xd8")
                    if start < 0:
                        buf = buf[-2:]
                        buf += pull()
                        continue
                    if start > 0:
                        buf = buf[start:]
                    end = buf.find(b"\xff\xd9", 2)
                    while end < 0:
                        buf += pull()
                        end = buf.find(b"\xff\xd9", 2)
                    jpg, buf = buf[:end + 2], buf[end + 2:]

                state["last_bytes"] = len(jpg)
                state["count"] += 1
                try:
                    frames.put_nowait(jpg)
                except queue.Full:
                    try:
                        frames.get_nowait()
                    except queue.Empty:
                        pass
                    frames.put_nowait(jpg)

        except Exception as exc:  # noqa: BLE001
            if state["stop"]:
                break
            state["status"] = "reconnecting"
            state["detail"] = f"{type(exc).__name__}: {exc}"
            log(f"断开（{state['detail']}），{RETRY_SEC}s 后重连")
            time.sleep(RETRY_SEC)
        finally:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    state["status"] = "closed"


_fullscreen = {"on": False}


def on_key(window, event):
    if event.keyval == Gdk.KEY_Escape:
        Gtk.main_quit()
    elif event.keyval in (Gdk.KEY_F11, Gdk.KEY_f):
        # 全屏 / 退出全屏
        if _fullscreen["on"]:
            window.unfullscreen()
        else:
            window.fullscreen()
        _fullscreen["on"] = not _fullscreen["on"]
    return False


def on_size_allocate(_widget, alloc):
    state["area"] = (max(alloc.width, 64), max(alloc.height, 48))


def fit(pixbuf, box_w, box_h):
    pw, ph = pixbuf.get_width(), pixbuf.get_height()
    scale = min(box_w / pw, box_h / ph)
    tw, th = max(1, int(pw * scale)), max(1, int(ph * scale))
    out = pixbuf.scale_simple(tw, th, GdkPixbuf.InterpType.BILINEAR)
    return out if out is not None else pixbuf


def frame_stats(pixbuf):
    """在解码后的原始帧上统计色块占比、亮度，供无人值守监看使用。"""
    w, h = pixbuf.get_width(), pixbuf.get_height()
    ch = pixbuf.get_n_channels()
    stride = pixbuf.get_rowstride()
    px = pixbuf.get_pixels()
    if ch < 3 or w < 8 or h < 8:
        return None

    total = red = blue = green = 0
    lum = 0.0
    sample = []
    for y in range(0, h, 4):
        base = y * stride
        for x in range(0, w, 4):
            i = base + x * ch
            r, g, b = px[i], px[i + 1], px[i + 2]
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


def tick(image, window):
    if state["stop"]:
        return False

    # 有画面就刷画面（重连期间保留最后一帧）
    try:
        jpg = frames.get_nowait()
    except queue.Empty:
        jpg = None
    if jpg is not None:
        loader = GdkPixbuf.PixbufLoader()
        try:
            loader.write(jpg)
            loader.close()
            pixbuf = loader.get_pixbuf()
            if pixbuf is not None:
                state["last_pixbuf"] = pixbuf
                box_w, box_h = state["area"]
                image.set_from_pixbuf(fit(pixbuf, box_w, box_h))
        except Exception:  # noqa: BLE001  坏帧直接跳过
            pass

    # 标题栏周期刷新（1s 一次，避免抖动），顺手把这一秒的画面统计写进监看日志
    now = time.time()
    if now - state["t0"] >= 1.0:
        elapsed = now - state["t0"]
        fps = state["count"] / elapsed if state["status"] == "connected" else 0.0
        state["count"] = 0
        state["t0"] = now
        if state["status"] == "connected":
            tag = "预览" if state["mode"] == "raw" else "比赛"
            window.set_title(f"beaconRace {HOST}:{PORT} [{tag}]   {state['last_bytes']}B   {fps:.1f}fps   连接#{state['connects']}")
        elif state["status"] in ("connecting", "reconnecting"):
            window.set_title(f"beaconRace {HOST}:{PORT}   {state['status']}...   {state['detail']}")
        else:
            window.set_title(f"beaconRace {HOST}:{PORT}   {state['status']}")

        # ---- 监看日志：解码后的真实帧统计 ----
        pb = state.get("last_pixbuf")
        s = frame_stats(pb) if pb is not None else None
        if s:
            d = diff_pct(state.get("prev_sample"), s["sample"])
            state["prev_sample"] = s["sample"]
            try:
                with open(STATUS_LOG, "a", encoding="utf-8") as fh:
                    fh.write(
                        "{ts}  {st:12s} {fps:4.1f}fps 红{red:6.2f}% 蓝{blue:6.2f}% "
                        "绿{green:6.2f}% 亮度{lum:6.1f} 变化{d:5.1f}%\n".format(
                            ts=time.strftime("%H:%M:%S"), st=state["status"], fps=fps,
                            d=d, **s))
            except OSError:
                pass
    return True


def main():
    window = Gtk.Window(title=f"beaconRace {HOST}:{PORT}")
    window.set_default_size(WIN_W, WIN_H)
    window.set_position(Gtk.WindowPosition.CENTER)
    window.connect("destroy", Gtk.main_quit)
    window.connect("key-press-event", on_key)

    image = Gtk.Image()
    image.set_hexpand(True)
    image.set_vexpand(True)
    image.set_halign(Gtk.Align.CENTER)
    image.set_valign(Gtk.Align.CENTER)
    # 固定最小尺寸：还没收到画面时窗口也不会缩成一小块
    image.set_size_request(WIN_W, WIN_H)
    window.add(image)
    window.connect("size-allocate", on_size_allocate)

    window.show_all()
    state["area"] = (window.get_allocated_width(), window.get_allocated_height())

    threading.Thread(target=reader, daemon=True).start()
    GLib.timeout_add(15, tick, image, window)

    try:
        with open(STATUS_LOG, "w", encoding="utf-8") as fh:
            fh.write(f"# viewer {HOST}:{PORT} 每 1s 一行画面统计\n")
    except OSError:
        pass

    log(f"窗口已开，等待 {HOST}:{PORT}（板子起 beaconRace 后自动接上）")
    Gtk.main()
    state["stop"] = True
    log("窗口已关闭")


if __name__ == "__main__":
    main()
