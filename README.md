# 穿越火线 · 2K0300 信标赛工具链

龙芯 2K0300（LoongOS / loongarch64）小车上运行的 `beaconRace` 比赛程序，以及围绕它的
编译、部署、监看脚本。板子地址 `192.168.1.3`（root，密钥登录）；交叉编译在虚拟机
`192.168.52.133`（mark）上进行，PC 只负责操作和转送。

## 关键事实

- **板子上没有编译器**，所有二进制都在虚拟机交叉编译后拷上去。
- 虚拟机工具链在 `/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6`，
  但没有 `cmake` / `make`，所以走裸 `g++` 编译脚本。
- 板子自带的 OpenCV **头文件是 4.10、库是 4.11**，`cvtColor` 签名不一致，靠
  `cv_compat_411.cpp` 补一个适配符号。
- 摄像头是独占设备：`beaconRace` 与独立预览服务不能同时占用 `/dev/video0`。
- 视频流协议：`beaconRace` 是「4 字节大端长度 + JPEG」，ffmpeg 预览是「裸 MJPEG」。

## 目录

| 文件 | 作用 |
|---|---|
| `deploy.ps1` | 一条命令：虚拟机编译 → 拉回 PC → 推上车 → 重启（逐跳 md5 校验、自动备份、可回滚） |
| `build_la.sh` | 虚拟机端裸 g++ 交叉编译 `beaconRace` |
| `cv_compat_411.cpp` | OpenCV 4.10 头文件 / 4.11 库的 `cvtColor` 适配层 |
| `board_swap.sh` | 板端备份并替换二进制 |
| `br_ctl.sh` | 板端启停控制器（`start [--drive]` / `stop` / `status` / `cam [--force]`） |
| `reset_comp.sh` | 按「成功方案」复位板上的二进制与配置 |
| `cam_preview.sh` / `cam_preview_loop.sh` | 独立摄像头预览（ffmpeg 推裸 MJPEG，默认不启用） |
| `gtk_viewer.py` | 虚拟机上位机窗口（GTK3，自动重连，自动识别两种流格式） |
| `vm_watch.sh` | 虚拟机端幂等启动上位机窗口 |
| `watch_run_all.ps1` | 全程监看一次实测：状态机 + 画面统计 → 事件时间线 |
| `viewer.ps1` / `capture_frame.ps1` | PC 端备用的零依赖播放器与抓帧工具 |
| `count_colors.py` / `verify_xwd.py` | 画面分析辅助脚本 |

## 常用命令

```powershell
# 编译并部署（默认 --no-drive，不动轮子）
powershell -File .\deploy.ps1
powershell -File .\deploy.ps1 -Mode drive     # 真驱动底盘
powershell -File .\deploy.ps1 -Restore        # 回滚到上一个备份

# 只起停，不编译
powershell -File .\stop_all.ps1               # 只停车程序，虚拟机窗口不动

# 全程监看一次实测
powershell -File .\watch_run_all.ps1 -MaxSeconds 900
```

```sh
# 板子上
sh /home/root/br_ctl.sh start [--drive] | stop | status
sh /home/root/cam_preview.sh start | stop | status

# 虚拟机上
sh /home/mark/vm_watch.sh
```

## 复位基线（成功方案）

| 项 | 值 |
|---|---|
| 二进制 | 141368 字节，md5 `871b0deafd151c11e44d0c280a1e9e99` |
| 配置 | 1783 字节，md5 `a8b0de8d45140fcc264d67ba419175b2`（42 项） |
| 源码 | `C:\Users\rdp\Downloads\成功方案\beaconRace.cpp`（58630 字节） |
