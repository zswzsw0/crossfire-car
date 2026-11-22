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

## 运行日志存档（重要）

**车上的 `/tmp/br_run.log` 是内存文件系统，重启/断电即失，且每次启动会被 `>` 清空。**
所以每次实测结束都必须把日志拉回 PC 存档。

| 文件 | 作用 |
|---|---|
| `save_run_log.ps1` | 手动存档：从车上拉 `/tmp/br_run.log` + 当时的 `beacon_race.cfg`，生成 `.log` / `.cfg` / `.meta.txt` 三件套，并提交 git |
| `watch_run_all.ps1` | 实测监看脚本，**结束时自动调用 `save_run_log.ps1`**，无需额外操作 |

```powershell
# 平时不用手动调，watch_run_all.ps1 结束会自动存档；
# 只有在没开监看、想补存当前日志时才需要手动：
powershell -NoProfile -ExecutionPolicy Bypass -File .\save_run_log.ps1 -Card blue -Result DONE -Seconds 18 -Note "goal 6.90 / tol 0.10"
```

存档命名：`run_logs/<日期>_<时分秒>_<卡色>_<结果>_<耗时>s.log`

`.meta.txt` 里记录：二进制 md5、卡色、结果、耗时、NAV_GOAL 行数、原地空转次数、最大 vx、
最终位姿、算好的终点误差、以及本轮备注。

## 复位基线（成功方案）

| 项 | 值 |
|---|---|
| 二进制 | 141368 字节，md5 `871b0deafd151c11e44d0c280a1e9e99` |
| 配置 | 1783 字节，md5 `a8b0de8d45140fcc264d67ba419175b2`（42 项） |
| 源码 | `C:\Users\rdp\Downloads\成功方案\beaconRace.cpp`（58630 字节） |

## 版本与 tag

| tag | 含义 | 关键参数 | 实测结果 |
|---|---|---|---|
| `v1.0-baseline` | 成功方案原样复位 | `vel_x_vision=0.15`、`goal_pos_tol=0.25` | 蓝卡 A 线终点误差 0.220 m；红卡 B 线见 `baseline/success_run.log` |
| `v1.1-blue-card-ok` | **蓝色卡慢速成功** | `vel_x_vision=0.30`、`goal_pos_tol=0.12` | 蓝卡 A 线 2026-11-18 10:12：终点 `(6.89363, -0.90012)`，误差 **0.106 m**，终点航向 1.41°，行驶段 53 s |
| **`v1.4`** | **稳定版：全阶段 1.5x + 入口航向收敛** | `vel_x_vision=0.675`、`vel_x_odom=1.35`、`vel_z_max=0.55`、`goal_pos_tol=0.12`、`ENTRY_ALIGN_DIST=1.80` | **6 轮全过**：蓝 A 误差 0.083/0.093 m、红 B 0.0806~0.1195 m；航向 1.1~6.9°；行驶 21~25 s；角速度零饱和 |

`v1.4` 是**唯一经过多轮实测验证的稳定基线**（二进制 md5 `78991d27`，配置快照 `cfg/beacon_race.fast225x.cfg`）。
之后的 `分区提速 1.0 / 2.0` 虽然更快（16~19 s），但暴露了新的问题（终点 $x$ 过冲、雷区撞锥），尚未定版。

`v1.1` 相对基线的全部差异（只有两项，均在 cfg 里，改完重启即可）：

```diff
-vel_x_vision=0.15
+vel_x_vision=0.30
-goal_pos_tol=0.25
+goal_pos_tol=0.12
```

参数含义与踩坑记录：

- `vel_x_vision` 是「识别到黑点、进入巡线模式」时的线速度。翻倍后实测只快约 5%，
  因为全程约 58% 的帧处于**丢线**状态（速度走 `LOST_VX=0.10` 兜底），近终点段
  （`d<1.80m`）又受编译期常量 `NEAR_VX_MAX=0.12` 限制。
- `goal_pos_tol` 原来是**停车判据**：`dist < goal_pos_tol` 直接 `DONE`。0.25 时车
  在营地门口就停了，后面的减速项 $v_x=\mathrm{clamp}(0.5d,0.05,0.12)$ 根本轮不到。
  改成 0.12 后车会带减速进入营地，实测末段出现 `vx=0.0818` 的减速档。
- 两者都**不需要重新编译**；编译期常量（`NEAR_VX_MAX`、`AVOID_YAW_*`、`LOST_VX`、
  `SPIN_YAW`、`CONE_PATH_HALF_W`、`CORRIDOR_*`）要改则必须走 `deploy.ps1`。

运行日志存档在 `run_logs/`。

