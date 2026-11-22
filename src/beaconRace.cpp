/* ============================================================================
 *  beaconRace.cpp  —— 参数改动指南（注释版）
 *
 *  本文件由板子上的原版源码复制而来，代码逻辑一字未改，只在关键位置插入注释。
 *
 *  ★★ 参数生效优先级（后面的覆盖前面的）★★
 *     1) 本文件内的默认值：常量区(原始第 32~53 行)、struct Config(原始第 58~103 行)、
 *        HSV 默认值(原始第 290~293 行)
 *     2) beacon_race.cfg —— 程序所在目录 与 当前工作目录 各读一次
 *     3) 命令行参数（--no-drive / --cam / --uart / --force-card / --no-stream）
 *
 *  ? 所以：板子上存在 beacon_race.cfg 时，改本文件的默认值【不会生效】！
 *     要改默认值生效，得删掉/改名 cfg，或者直接改 cfg，或用 --config 另指一个。
 *
 *  ★ 改完必须重新交叉编译并拷回板子（板子上没有编译器）★
 *     cd ~/Downloads/2K0300_deepbot/99pai_MEC_Car/serial
 *     cd build_cross && make beaconRace -j2
 *     产物：build_cross/beaconRace/beaconRace
 *     拷到板子后运行时若 OpenCV 报错，加环境变量：
 *     LD_LIBRARY_PATH=/home/root/opencv/lib ./beaconRace
 *
 *  ★ 常改参数速查（用 Ctrl+F 搜标号跳转；括号里是原始文件行号）★
 *     速度/角速度/增益   → 搜索 B-6  （原始第 92~102 行）
 *     据点与终点坐标     → 搜索 B-5  （原始第 87~90 行）
 *     色卡判定门槛       → 搜索 B-2  （原始第 68~71 行）
 *     黑点面积范围       → 搜索 B-3  （原始第 73~74 行）
 *     锥桶面积/避让      → 搜索 B-4  （原始第 76~85 行）
 *     ROI 高度           → 搜索 B-2  （原始第 68 行）
 *     HSV 色域阈值       → 搜索 C    （原始第 290~293 行）
 *     丢线/找线控制量    → 搜索 A-7  （原始第 49~53 行）
 *     图像分辨率         → 搜索 A-1  （原始第 32~33 行）
 *     cfg 加载顺序陷阱   → 搜索 「关键陷阱」
 * ========================================================================== */

// 穿越火线。逻辑对应 catkin_ws 的 beacon_race.py / beacon_race.launch。
// 底盘指令走 99pai 串口协议 sendVelocity(vx, vy, wz)，与 deepbot_driver::SetVelocity 同一帧格式。
// 比赛只使用 vx 和 wz，vy 保持 0。里程计用底盘回传速度积分，航向默认用陀螺仪 z。

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <unistd.h>

#include "camera.hpp"
#include "carProtocol.hpp"
#include "imgStream.hpp"
#include "keyboard.hpp"
#include "serialPort.hpp"

// ┌────────────────────────────────────────────────────────────────────────┐
// │ 【改动点 A】硬编码常量区 —— 这一组 cfg 里没有，改了必须重新编译        │
// │ 想让它们变成可配置项，需要在 struct Config 加字段 + 在 applyKey() 加解析 │
// └────────────────────────────────────────────────────────────────────────┘
// A-1 图像处理分辨率：相机按此尺寸初始化(原始第 300 行)，每帧又 resize 到同样尺寸(第 329 行)。
//     调大 → 看得更远，但 2K0300 算力吃力、帧率掉；调小 → 更快更稳。
//     改这里会连带影响 ROI 高度(第 287 行)和画面文字位置，三者要一起考虑。
static constexpr int IMG_W = 320;
static constexpr int IMG_H = 240;
static constexpr double CENTER_X = IMG_W * 0.5;
static constexpr double ODOM_DT = 0.02; // deepbot_driver DATA_PERIOD
static constexpr double kPi = 3.14159265358979323846;

// A-2 形态学核大小：黑点 mask 噪点多就调大(7/9)，黑点被腐蚀掉就调小(3)。
static constexpr int MORPH_KERNEL = 5;
// A-3 黑点几何筛选：长宽比范围 + 最小宽高(像素)。
//     小黑点漏检 → 调小 MIN_BLACK_W/MIN_BLACK_H；把噪点当黑点 → 调大。
static constexpr double MIN_BLACK_ASPECT = 0.20;
static constexpr double MAX_BLACK_ASPECT = 2.2;
static constexpr int MIN_BLACK_W = 8;
static constexpr int MIN_BLACK_H = 6;
// A-4 同一行黑点合并规则：Y 方向容差比例 + 面积占比门槛。
//     一个黑点被拆成多个 → 调大 ROW_Y_RATIO。
static constexpr double ROW_Y_RATIO = 0.14;
static constexpr double ROW_AREA_RATIO = 0.70;
// A-5 巡线用黑点间距的合理范围(像素)。场地尺寸或相机安装高度变了就调这里。
static constexpr int MIN_BLACK_GAP_PX = 15;
static constexpr int MAX_BLACK_GAP_PX = 160;
// A-6 锥桶长宽比范围：相机压得低时锥桶画面更细长，需要调大 CONE_MAX_ASPECT。
static constexpr double CONE_MIN_ASPECT = 0.6;
static constexpr double CONE_MAX_ASPECT = 3.5;
// A-7 ★ 控制量 ★ 直接决定车的动作幅度：
//     SPIN_YAW    看不到线时原地找线的角速度(rad/s)
//     LOST_YAW    丢线时的转向角速度
//     NEAR_VX_MAX 接近目标阶段的限速
//     NEAR_VX_P   接近目标阶段的距离比例系数
//     LOST_VX     丢线时仍保持的前进速度
//     车抖/画龙 → 减小 SPIN_YAW、LOST_YAW；反应迟钝 → 适当增大。
static constexpr double SPIN_YAW = 0.6;
static constexpr double LOST_YAW = 0.7;
static constexpr double NEAR_VX_MAX = 0.27;   // ×1.5（再乘 1.5，原 0.18）：近终点段限速
static constexpr double NEAR_VX_P = 1.125;    // ×1.5（再乘 1.5，原 0.75）：近终点距离比例系数
static constexpr double LOST_VX = 0.225;      // ×1.5（再乘 1.5，原 0.15）：丢线兜底速度

// ★ 终点入口收敛（按场地图：营地入口正对赛道，车应平着沿 +x 进入）
//   速度再快 1.5 倍 → 收敛窗口与摆正时间同比例放大，否则会擦过目标点
static constexpr double ENTRY_ALIGN_DIST  = 1.80;  // 距终点多远开始混入航向收敛(m)，原 1.20
static constexpr double ENTRY_ALIGN_W     = 0.85;  // 航向收敛的最大权重(0~1)，原 0.75
static constexpr double GOAL_ALIGN_MAX_SEC= 6.0;   // 到点后原地摆正航向的最长时间(s)，原 4.0

// ★★★ 本次新增：避障限时 + 走廊约束（解决"撞锥桶"与"跑出场地"两个矛盾）★★★
static constexpr int    AVOID_MAX_FRAMES = 45;    // 单次避让最多持续帧数，30fps 下约 1.5s
static constexpr double AVOID_YAW_MAX    = 0.65;  // ★ 避让时航向最大偏置(rad, 约37°)，有界，防止绕圈
static constexpr double AVOID_YAW_RATE   = 0.030; // 每帧向偏置角靠近的步长
static constexpr double AVOID_YAW_RECOVER= 0.015; // 无障碍时每帧回正步长
static constexpr double CORRIDOR_HALF_W  = 0.55;  // 允许偏离 base→goal 直线的横向半宽(米)
static constexpr double CORRIDOR_GAIN    = 1.2;   // 超出走廊后的回归角速度增益
static constexpr double CONE_PATH_HALF_W = 60.0;  // 锥桶横向偏离行进线的基础阈值(px)，实际按远近自动放大

static std::atomic<bool> g_running{true};
static void signalHandler(int) { g_running = false; }

// ┌────────────────────────────────────────────────────────────────────────┐
// │ 【改动点 B】struct Config 默认值 —— 绝大多数调参都在这一段             │
// │ ? 只是默认值，beacon_race.cfg 里同名项会覆盖(见文件末尾「关键陷阱」)    │
// │ 每个字段在 applyKey()(原始第 156 行起) 对应一个同名 cfg 键，            │
// │ 例如 yaw_p 对应 cfg 里的一行：  yaw_p 1.2                              │
// └────────────────────────────────────────────────────────────────────────┘
struct Config {
// B-1 运行开关（都可用命令行覆盖）：
//     enable_drive      false 等价 --no-drive（只跑视觉不发速度，调试必用）
//     allow_force_card  true  等价 --force-card（允许按 b/r 手动选色）
//     use_gyro_yaw      航向用陀螺仪 z 还是纯积分
//     stream_enable     false 等价 --no-stream（关掉 TCP 图传）
//     stream_port       就是 viewer.py 连的 9999
//     uart              串口设备；板子 /dev/ttyUSB0，ROS 驱动 /dev/deepbotbase
//     hsv_file          调色保存文件，存在时会覆盖下面的 HSV 默认值
    bool enable_drive = true;
    bool allow_force_card = false;
    bool use_gyro_yaw = true;
    bool stream_enable = true;
    int camera_id = 0;
    int stream_port = 9999;
    std::string uart = "/dev/ttyUSB0";
    std::string hsv_file = "hsv_saved.txt";

// B-2 ROI 与色卡判定门槛：
//     roi_y_top = 0.45      画面上方 45% 不参与检测，只在地面找黑点/锥桶。
//                           黑点在画面里偏上 → 调小；只想看脚下 → 调大。
//     card_fill_ratio=0.50  色卡要占整幅画面 ≥50% 才认。误触发就调大(0.6~0.7)。
//     card_confirm_frames=12 连续满足多少帧才确认，防抖。启动太容易误判就调大。
//     card_dominance=1.50   主色面积须是另一色的 1.5 倍，避免红蓝同时出现时误判。
    double roi_y_top = 0.45;
    double card_fill_ratio = 0.50;
    int card_confirm_frames = 12;
    double card_dominance = 1.50;

// B-3 黑点面积范围(像素)：滤掉噪点、也滤掉大片阴影和色卡边缘。
//     小黑点漏检 → 调小 min_black_area；阴影被当成地面点 → 调大 max_black_area。
    double min_black_area = 20;
    double max_black_area = 700;

// B-4 锥桶检测与避让：
//     red_cone_min/max_area   红锥面积范围，按实际画面标定
//     red_avoid_wz/vx         遇红锥的转向角速度/前进速度（红锥要真避，参数硬）
//     blue_cone_min/max_area  蓝锥面积范围
//     blue_avoid_wz/vx        蓝锥只做轻避
//     blue_on_track_dist_px=40 只有蓝锥中心距黑点连线 ≤40px 才触发轻避
    double red_cone_min_area = 200;
    double red_cone_max_area = 1400;
    double red_avoid_wz = 0.45;
    double red_avoid_vx = 0.08;

    double blue_cone_min_area = 240;
    double blue_cone_max_area = 1400;
    double blue_avoid_wz = 0.20;
    double blue_avoid_vx = 0.10;
    double blue_on_track_dist_px = 40;

// B-5 ★★★ 最常改：据点与终点坐标 ★★★
//     坐标在「起点坐标系」下，单位米，车头方向 = +X，左侧 = +Y。
//       蓝卡路线：据点 A(blue_base_x,y) → 终点(blue_goal_x,y)
//       红卡路线：据点 B(red_base_x,y)  → 终点(red_goal_x,y)
//     场地尺寸变了、起点位置平移了，就改这四个点。
//     ? 改完请同步到 beacon_race.cfg，否则 cfg 会覆盖本文件的值。
    double blue_base_x = 1.0, blue_base_y = 0.9;
    double red_base_x = 1.0, red_base_y = -0.9;
    double blue_goal_x = 7.0, blue_goal_y = -0.9;
    double red_goal_x = 7.0, red_goal_y = 0.9;

// B-6 ★★★ 最常改：速度与闭环增益 ★★★
//     vel_x_odom=0.20     里程计导航段直线速度(m/s)  ← 车跑太慢先加这个
//     vel_x_vision=0.15   视觉巡线段前进速度(m/s)
//     vel_z_max=0.7       角速度上限(rad/s)，车抖/画龙就降
//     yaw_p=1.2           航向比例系数 ← 朝向纠偏力度
//     pos_p=0.8           位置比例系数（离目标远时按距离给速度）
//     goal_pos_tol=0.18   到点判定半径(m)，停不准调小、末段抖动调大
//     goal_yaw_tol=0.12   到点航向容差(rad)
//     base_wait_sec=3.0   据点处停车等待秒数
//     near_goal_dist=1.2  距终点多少米内切换成视觉巡线
//     vis_wz_gain=0.008   视觉横向误差 → 角速度的增益
//     odom_blend=0.35     视觉与里程计航向的融合权重
    double vel_x_odom = 0.60;
    double vel_x_vision = 0.15;
    double vel_z_max = 0.7;
    double yaw_p = 1.2;
    double pos_p = 0.8;
    double goal_pos_tol = 0.18;
    double goal_yaw_tol = 0.12;
    double base_wait_sec = 3.0;
    double near_goal_dist = 1.2;
    double vis_wz_gain = 0.008;
    double odom_blend = 0.35;
};

struct Blob {
    double cx = 0, cy = 0, w = 0, h = 0, area = 0;
};

struct Track {
    std::vector<Blob> blobs;
    double mid_x = 0, mid_y = 0;
};

enum class Phase { WaitCard, NavBase, WaitBase, NavGoal, Done };

static const char* phaseName(Phase p)
{
    switch (p) {
    case Phase::WaitCard: return "WAIT_CARD";
    case Phase::NavBase:  return "NAV_BASE";
    case Phase::WaitBase: return "WAIT_BASE";
    case Phase::NavGoal:  return "NAV_GOAL";
    case Phase::Done:     return "DONE";
    }
    return "?";
}

static double wrapPi(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

static double clampd(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

static std::string trim(std::string s)
{
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    return s;
}

static bool parseBool(const std::string& v)
{
    std::string s;
    for (char c : v) s.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    if (s == "1" || s == "true" || s == "yes" || s == "on") return true;
    if (s == "0" || s == "false" || s == "no" || s == "off") return false;
    throw std::runtime_error("not a bool");
}

static bool applyKey(Config& c, const std::string& k, const std::string& v)
{
    try {
        if (k == "enable_drive") { c.enable_drive = parseBool(v); return true; }
        if (k == "allow_force_card") { c.allow_force_card = parseBool(v); return true; }
        if (k == "use_gyro_yaw") { c.use_gyro_yaw = parseBool(v); return true; }
        if (k == "stream_enable") { c.stream_enable = parseBool(v); return true; }
        if (k == "camera_id") { c.camera_id = std::stoi(v); return true; }
        if (k == "stream_port") { c.stream_port = std::stoi(v); return true; }
        if (k == "uart") { c.uart = v; return true; }
        if (k == "hsv_file") { c.hsv_file = v; return true; }
        if (k == "roi_y_top") { c.roi_y_top = std::stod(v); return true; }
        if (k == "card_fill_ratio") { c.card_fill_ratio = std::stod(v); return true; }
        if (k == "card_confirm_frames") { c.card_confirm_frames = std::stoi(v); return true; }
        if (k == "card_dominance") { c.card_dominance = std::stod(v); return true; }
        if (k == "min_black_area") { c.min_black_area = std::stod(v); return true; }
        if (k == "max_black_area") { c.max_black_area = std::stod(v); return true; }
        if (k == "red_cone_min_area") { c.red_cone_min_area = std::stod(v); return true; }
        if (k == "red_cone_max_area") { c.red_cone_max_area = std::stod(v); return true; }
        if (k == "red_avoid_wz") { c.red_avoid_wz = std::stod(v); return true; }
        if (k == "red_avoid_vx") { c.red_avoid_vx = std::stod(v); return true; }
        if (k == "blue_cone_min_area") { c.blue_cone_min_area = std::stod(v); return true; }
        if (k == "blue_cone_max_area") { c.blue_cone_max_area = std::stod(v); return true; }
        if (k == "blue_avoid_wz") { c.blue_avoid_wz = std::stod(v); return true; }
        if (k == "blue_avoid_vx") { c.blue_avoid_vx = std::stod(v); return true; }
        if (k == "blue_on_track_dist_px") { c.blue_on_track_dist_px = std::stod(v); return true; }
        if (k == "blue_base_x") { c.blue_base_x = std::stod(v); return true; }
        if (k == "blue_base_y") { c.blue_base_y = std::stod(v); return true; }
        if (k == "red_base_x") { c.red_base_x = std::stod(v); return true; }
        if (k == "red_base_y") { c.red_base_y = std::stod(v); return true; }
        if (k == "blue_goal_x") { c.blue_goal_x = std::stod(v); return true; }
        if (k == "blue_goal_y") { c.blue_goal_y = std::stod(v); return true; }
        if (k == "red_goal_x") { c.red_goal_x = std::stod(v); return true; }
        if (k == "red_goal_y") { c.red_goal_y = std::stod(v); return true; }
        if (k == "vel_x_odom") { c.vel_x_odom = std::stod(v); return true; }
        if (k == "vel_x_vision") { c.vel_x_vision = std::stod(v); return true; }
        if (k == "vel_z_max") { c.vel_z_max = std::stod(v); return true; }
        if (k == "yaw_p") { c.yaw_p = std::stod(v); return true; }
        if (k == "pos_p") { c.pos_p = std::stod(v); return true; }
        if (k == "goal_pos_tol") { c.goal_pos_tol = std::stod(v); return true; }
        if (k == "goal_yaw_tol") { c.goal_yaw_tol = std::stod(v); return true; }
        if (k == "base_wait_sec") { c.base_wait_sec = std::stod(v); return true; }
        if (k == "near_goal_dist") { c.near_goal_dist = std::stod(v); return true; }
        if (k == "vis_wz_gain") { c.vis_wz_gain = std::stod(v); return true; }
        if (k == "odom_blend") { c.odom_blend = std::stod(v); return true; }
    } catch (const std::exception& e) {
        std::cerr << "配置项数值无效 " << k << "=" << v << " (" << e.what() << ")\n";
        return true;
    }
    return false;
}

static bool loadConfigFile(const std::string& path, Config& c, bool required)
{
    std::ifstream in(path);
    if (!in) {
        if (required) std::cerr << "打不开配置文件: " << path << "\n";
        return false;
    }
    std::string line;
    int n = 0;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        auto eq = line.find('=');
        if (eq == std::string::npos) {
            std::cerr << "忽略无法解析的行: " << line << "\n";
            continue;
        }
        std::string k = trim(line.substr(0, eq));
        std::string v = trim(line.substr(eq + 1));
        if (!applyKey(c, k, v)) std::cerr << "未知配置项: " << k << "\n";
        else ++n;
    }
    std::cout << "已读取配置 " << path << " (" << n << " 项)\n";
    return true;
}

static std::string exeDir()
{
    char buf[4096];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    buf[n] = 0;
    std::string s(buf);
    auto p = s.rfind('/');
    return p == std::string::npos ? std::string(".") : s.substr(0, p);
}

static void clampHue(std::array<int, 6>& h)
{
    for (int i = 0; i < 6; ++i) {
        int mx = (i == 0 || i == 3) ? 180 : 255;
        h[i] = std::max(0, std::min(mx, h[i]));
    }
}

static bool parseHueBody(const std::string& body, std::array<int, 6>& out)
{
    std::string s = body;
    for (char& c : s) if (c == ',') c = ' ';
    std::istringstream ss(s);
    for (int i = 0; i < 6; ++i) if (!(ss >> out[i])) return false;
    clampHue(out);
    return true;
}

static double pointSegDist(double px, double py, double ax, double ay, double bx, double by)
{
    double abx = bx - ax, aby = by - ay;
    double apx = px - ax, apy = py - ay;
    double ab2 = abx * abx + aby * aby;
    if (ab2 < 1e-6) return std::hypot(apx, apy);
    double t = clampd((apx * abx + apy * aby) / ab2, 0.0, 1.0);
    return std::hypot(px - (ax + t * abx), py - (ay + t * aby));
}

static std::vector<std::vector<cv::Point>> contoursOf(const cv::Mat& mask)
{
    cv::Mat work = mask.clone();
    std::vector<std::vector<cv::Point>> cs;
    cv::findContours(work, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return cs;
}

class BeaconRace {
public:
    BeaconRace(SerialPort* uart, Config cfg)
        : uart_(uart), cfg_(std::move(cfg))
    {
        roi_y_ = static_cast<int>(IMG_H * cfg_.roi_y_top);
        roi_y_ = std::max(0, std::min(IMG_H, roi_y_));
        motion_enabled_ = cfg_.enable_drive;
// ┌────────────────────────────────────────────────────────────────────────┐
// │ 【改动点 C】HSV 色域默认值 —— 格式 {Hmin, Smin, Vmin, Hmax, Smax, Vmax} │
// │ 这里只是出厂默认；只要程序目录下存在 hsv_saved.txt，就会被它覆盖        │
// │ (loadHsv()，原始第 896 行)。                                          │
// │ 所以推荐做法：不改这里，而是运行程序后按 f 现场调、按 q 保存。          │
// │ 参考范围：蓝色赛道 H 100~130；黄色 20~40；红色 0~15 或 156~180；       │
// │           白色斑马线 S 很低(0~30)。                                    │
// └────────────────────────────────────────────────────────────────────────┘
        hue_black_ = {{0, 0, 0, 180, 80, 60}};
        hue_red_1_ = {{0, 80, 60, 10, 255, 255}};
        hue_red_2_ = {{156, 80, 60, 180, 255, 255}};
        hue_blue_  = {{100, 80, 46, 124, 255, 255}};
        loadHsv(false);
    }

    int run()
    {
        Camera cam;
// C-2 相机初始化：第 2 个参数是目标帧率 60fps（实际受 USB 带宽/光照限制）
//     后两个参数是采集分辨率 IMG_W×IMG_H。改分辨率要连 A-1 一起改，
//     否则第 329 行还会再 resize 一次（多做一次无用开销）。
        if (!cam.init(static_cast<uint8_t>(cfg_.camera_id), 60.0, IMG_W, IMG_H)) {
            std::cerr << "打开摄像头失败 id=" << cfg_.camera_id << "\n";
            return 1;
        }
        ImgStream stream(cfg_.stream_port, cfg_.stream_enable);
        keyboardInit();
        struct KbGuard { ~KbGuard() { keyboardRestore(); } } guard;

        std::cout << "beaconRace 启动 | " << phaseName(phase_)
                  << " | drive=" << (cfg_.enable_drive ? "on" : "off")
                  << " force=" << (cfg_.allow_force_card ? "on" : "off")
                  << " gyro_yaw=" << (cfg_.use_gyro_yaw ? "on" : "off") << "\n"
                  << "蓝卡 A(" << cfg_.blue_base_x << "," << cfg_.blue_base_y
                  << ") -> (" << cfg_.blue_goal_x << "," << cfg_.blue_goal_y << ")\n"
                  << "红卡 B(" << cfg_.red_base_x << "," << cfg_.red_base_y
                  << ") -> (" << cfg_.red_goal_x << "," << cfg_.red_goal_y << ")\n"
                  << "按键: f改HSV q保存 l重载 x停车/恢复 v切换画面 c退出"
                  << (cfg_.allow_force_card ? " b/r强制色卡" : "") << "\n";

        cv::Mat frame, image, hsv;
        int miss = 0;
        auto last_log = std::chrono::steady_clock::now();
        while (g_running) {
            if (!cam.read(frame) || frame.empty()) {
                if (++miss > 10) sendStop();
                continue;
            }
            miss = 0;
            pollOdom();
// C-3 每帧统一缩放到 IMG_W×IMG_H 再做 HSV 变换，后续所有阈值都是按这个尺度标定的。
            cv::resize(frame, image, cv::Size(IMG_W, IMG_H), 0, 0, cv::INTER_AREA);
            cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

            cv::Mat mask_k, mask_r, mask_b;
            auto blacks = detectBlacks(hsv, mask_k);
            auto reds = detectCones(hsv, "red", cfg_.red_cone_min_area, cfg_.red_cone_max_area, mask_r);
            auto blues = detectCones(hsv, "blue", cfg_.blue_cone_min_area, cfg_.blue_cone_max_area, mask_b);

            bool has_track = false;
            Track track;
            if (findBlackTrack(blacks, track)) has_track = true;

            handleKeys();
            if (!g_running) break;

            Blob avoid_red, avoid_blue;
            bool has_red = false, has_blue = false;
            if (phase_ == Phase::NavGoal) {
                has_red = pickConeInPath(reds, cfg_.red_cone_min_area, has_track ? &track : nullptr, avoid_red);
                has_blue = pickConeInPath(blues, cfg_.blue_cone_min_area, has_track ? &track : nullptr, avoid_blue);
            }
            cv::Rect card_box;
            bool has_card = false;
            update(hsv, has_track ? &track : nullptr,
                   has_red ? &avoid_red : nullptr,
                   has_blue ? &avoid_blue : nullptr,
                   card_box, has_card);

            draw(image, blacks, reds, blues, has_track ? &track : nullptr, has_card ? &card_box : nullptr);
            if (view_ == 0) stream.push(image);
            else if (view_ == 1) stream.push(mask_k);
            else if (view_ == 2) stream.push(mask_r);
            else stream.push(mask_b);

            auto now = std::chrono::steady_clock::now();
            if (now - last_log > std::chrono::seconds(1)) {
                last_log = now;
                std::cout << phaseName(phase_)
                          << " card=" << (card_color_.empty() ? "-" : card_color_)
                          << " pose=(" << x_ << "," << y_ << ")"
                          << " yaw=" << (yaw_ * 180.0 / kPi) << "deg"
                          << " vx=" << cmd_vx_ << " wz=" << cmd_wz_
                          << " err=" << erro_
                          << " odom=" << (have_odom_ ? "ok" : "wait")
                          << " " << card_label_ << "\n";
            }
        }
        sendStop();
        cam.release();
        return 0;
    }

private:
    SerialPort* uart_;
    Config cfg_;
    int roi_y_ = 0;
    bool motion_enabled_ = true;
    int view_ = 0;

    std::array<int, 6> hue_black_{}, hue_red_1_{}, hue_red_2_{}, hue_blue_{};

    Phase phase_ = Phase::WaitCard;
    std::string card_color_;
    std::string card_label_;
    int hold_blue_ = 0, hold_red_ = 0;
    int avoid_frames_ = 0;   // ★ 连续避让帧计数，用于限时
    double avoid_yaw_ = 0;   // ★ 避让航向偏置(rad)，有界，红卡取正(左)、蓝卡取负(右)
    double base_x_ = 0, base_y_ = 0, goal_x_ = 0, goal_y_ = 0;
    std::chrono::steady_clock::time_point wait_t0_{};
    double erro_ = 0;

    bool have_odom_ = false;
    uint64_t last_seq_ = 0;
    double x_ = 0, y_ = 0, yaw_ = 0;
    double gyro_bias_ = 0, bias_sum_ = 0;
    int bias_n_ = 0;

    double cmd_vx_ = 0, cmd_wz_ = 0;
    bool avoiding_ = false;
    std::string avoid_side_, avoid_color_;
    Blob avoid_blob_{};
    bool goal_aligning_ = false;                       // ★ 已到点、正在原地摆正航向
    std::chrono::steady_clock::time_point goal_align_t0_{};  // ★ 开始摆正的时刻

    void send(double vx, double wz) { send(vx, 0.0, wz); }

    // ★ 本次新增 vy 重载：麦轮底盘可原地横移，避障用横移代替转向
    void send(double vx, double vy, double wz)
    {
        bool allow = cfg_.enable_drive && motion_enabled_
            && phase_ != Phase::WaitCard
            && phase_ != Phase::WaitBase
            && phase_ != Phase::Done;
        if (!allow) { vx = 0; vy = 0; wz = 0; }
        wz = clampd(wz, -std::abs(cfg_.vel_z_max), std::abs(cfg_.vel_z_max));
        cmd_vx_ = vx;
        cmd_wz_ = wz;
        if (uart_) sendVelocity(uart_, vx, vy, wz);
    }

    void sendStop() { send(0, 0); }

    void pollOdom()
    {
        uint64_t seq = g_sensorFrames.load(std::memory_order_acquire);
        if (seq == 0) return;
        have_odom_ = true;
        uint64_t n = seq - last_seq_;
        last_seq_ = seq;
        if (n == 0) return;
        SensorData sd = getLatestSensor();

        if (phase_ == Phase::WaitCard) {
            if (cfg_.use_gyro_yaw) {
                bias_sum_ += sd.gyro_z;
                ++bias_n_;
                gyro_bias_ = bias_sum_ / bias_n_;
            }
            x_ = y_ = yaw_ = 0;
            return;
        }

        if (n > 25) n = 25;
        double wz = cfg_.use_gyro_yaw ? (sd.gyro_z - gyro_bias_) : sd.angular_z;
        if (cfg_.use_gyro_yaw && std::abs(wz) < 0.01) wz = 0.0;
        for (uint64_t i = 0; i < n; ++i) {
            double c = std::cos(yaw_);
            double sn = std::sin(yaw_);
            x_ += (sd.linear_x * c - sd.linear_y * sn) * ODOM_DT;
            y_ += (sd.linear_x * sn + sd.linear_y * c) * ODOM_DT;
            yaw_ = wrapPi(yaw_ + wz * ODOM_DT);
        }
    }

    cv::Mat maskOf(const cv::Mat& hsv, const std::array<int, 6>& hue) const
    {
        cv::Mat m;
        cv::inRange(hsv, cv::Scalar(hue[0], hue[1], hue[2]),
                    cv::Scalar(hue[3], hue[4], hue[5]), m);
        return m;
    }

    cv::Mat redMask(const cv::Mat& hsv) const
    {
        cv::Mat a = maskOf(hsv, hue_red_1_);
        cv::Mat b = maskOf(hsv, hue_red_2_);
        cv::Mat o;
        cv::bitwise_or(a, b, o);
        return o;
    }

    void zeroRoi(cv::Mat& mask) const
    {
        if (roi_y_ <= 0 || mask.empty()) return;
        int h = std::min(roi_y_, mask.rows);
        mask(cv::Rect(0, 0, mask.cols, h)).setTo(0);
    }

    struct CardStat { int fill = 0; double ratio = 0; bool has_box = false; cv::Rect box; };

    CardStat cardStats(const cv::Mat& hsv, const std::string& color) const
    {
        cv::Mat mask = (color == "blue") ? maskOf(hsv, hue_blue_) : redMask(hsv);
        cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);
        CardStat st;
        st.fill = cv::countNonZero(mask);
        st.ratio = st.fill / static_cast<double>(IMG_W * IMG_H);
        double best = 0;
        for (const auto& c : contoursOf(mask)) {
            double a = cv::contourArea(c);
            if (a > best) {
                best = a;
                st.box = cv::boundingRect(c);
                st.has_box = true;
            }
        }
        return st;
    }

    std::vector<Blob> detectBlacks(const cv::Mat& hsv, cv::Mat& mask) const
    {
        mask = maskOf(hsv, hue_black_);
        zeroRoi(mask);
        cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_KERNEL, MORPH_KERNEL));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);
        std::vector<Blob> blobs;
        for (const auto& c : contoursOf(mask)) {
            double area = cv::contourArea(c);
            if (area < cfg_.min_black_area || area > cfg_.max_black_area) continue;
            cv::Rect r = cv::boundingRect(c);
            if (r.height < MIN_BLACK_H || r.width < MIN_BLACK_W) continue;
            double asp = static_cast<double>(r.height) / static_cast<double>(r.width);
            if (asp < MIN_BLACK_ASPECT || asp > MAX_BLACK_ASPECT) continue;
            blobs.push_back(Blob{r.x + r.width * 0.5, r.y + r.height * 0.5,
                                 static_cast<double>(r.width), static_cast<double>(r.height), area});
        }
        return blobs;
    }

    std::vector<Blob> detectCones(const cv::Mat& hsv, const char* color,
                                  double min_area, double max_area, cv::Mat& mask) const
    {
        mask = (std::strcmp(color, "blue") == 0) ? maskOf(hsv, hue_blue_) : redMask(hsv);
        zeroRoi(mask);
        cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(MORPH_KERNEL, MORPH_KERNEL));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);
        std::vector<Blob> blobs;
        for (const auto& c : contoursOf(mask)) {
            double area = cv::contourArea(c);
            if (area < min_area || area > max_area) continue;
            cv::Rect r = cv::boundingRect(c);
            if (r.height < 1 || r.width < 1) continue;
            double asp = static_cast<double>(r.height) / static_cast<double>(r.width);
            if (asp < CONE_MIN_ASPECT || asp > CONE_MAX_ASPECT) continue;
            blobs.push_back(Blob{r.x + r.width * 0.5, r.y + r.height * 0.5,
                                 static_cast<double>(r.width), static_cast<double>(r.height), area});
        }
        return blobs;
    }

    static std::vector<std::vector<Blob>> clusterRows(const std::vector<Blob>& blobs)
    {
        if (blobs.empty()) return {};
        std::vector<Blob> remaining = blobs;
        std::sort(remaining.begin(), remaining.end(),
                  [](const Blob& a, const Blob& b) { return a.area > b.area; });
        std::vector<std::vector<Blob>> rows;
        while (!remaining.empty()) {
            Blob seed = remaining.front();
            remaining.erase(remaining.begin());
            std::vector<Blob> row{seed}, kept;
            for (const auto& b : remaining) {
                bool same = std::abs(b.area - seed.area) / std::max(seed.area, 1.0) <= ROW_AREA_RATIO
                    && std::abs(b.cy - seed.cy) / static_cast<double>(IMG_H) <= ROW_Y_RATIO;
                if (same) row.push_back(b);
                else kept.push_back(b);
            }
            remaining.swap(kept);
            std::sort(row.begin(), row.end(),
                      [](const Blob& a, const Blob& b) { return a.cx < b.cx; });
            rows.push_back(std::move(row));
        }
        std::sort(rows.begin(), rows.end(), [](const std::vector<Blob>& a, const std::vector<Blob>& b) {
            auto mean = [](const std::vector<Blob>& r) {
                double s = 0;
                for (const auto& it : r) s += it.cy;
                return s / static_cast<double>(r.size());
            };
            return mean(a) > mean(b);
        });
        return rows;
    }

    bool findBlackTrack(const std::vector<Blob>& blacks, Track& best) const
    {
        if (blacks.size() < 2) return false;
        bool any = false;
        double best_score = 0;
        for (const auto& row : clusterRows(blacks)) {
            if (row.size() >= 3) {
                for (size_t i = 0; i + 2 < row.size(); ++i) {
                    double g1 = row[i + 1].cx - row[i].cx;
                    double g2 = row[i + 2].cx - row[i + 1].cx;
                    if (g1 < MIN_BLACK_GAP_PX || g1 > MAX_BLACK_GAP_PX) continue;
                    if (g2 < MIN_BLACK_GAP_PX || g2 > MAX_BLACK_GAP_PX) continue;
                    double mx = (row[i].cx + row[i + 1].cx + row[i + 2].cx) / 3.0;
                    double my = (row[i].cy + row[i + 1].cy + row[i + 2].cy) / 3.0;
                    double score = 200.0 - 0.4 * std::abs(mx - CENTER_X) + 0.15 * my;
                    if (!any || score > best_score) {
                        best.blobs = {row[i], row[i + 1], row[i + 2]};
                        best.mid_x = mx;
                        best.mid_y = my;
                        best_score = score;
                        any = true;
                    }
                }
            }
            if (row.size() >= 2) {
                for (size_t i = 0; i + 1 < row.size(); ++i) {
                    double gap = row[i + 1].cx - row[i].cx;
                    if (gap < MIN_BLACK_GAP_PX || gap > MAX_BLACK_GAP_PX) continue;
                    double mx = 0.5 * (row[i].cx + row[i + 1].cx);
                    double my = 0.5 * (row[i].cy + row[i + 1].cy);
                    double score = 100.0 - 0.4 * std::abs(mx - CENTER_X) + 0.15 * my;
                    if (!any || score > best_score) {
                        best.blobs = {row[i], row[i + 1]};
                        best.mid_x = mx;
                        best.mid_y = my;
                        best_score = score;
                        any = true;
                    }
                }
            }
        }
        return any;
    }

    static bool pickNearest(const std::vector<Blob>& cones, double min_area, Blob& out)
    {
        bool any = false;
        for (const auto& b : cones) {
            if (b.area < min_area) continue;
            if (!any || b.cy > out.cy || (b.cy == out.cy && b.area > out.area)) {
                out = b;
                any = true;
            }
        }
        return any;
    }

    bool coneOnTrack(const Blob& cone, const Track& track) const
    {
        if (track.blobs.size() < 2) return false;
        double thr = cfg_.blue_on_track_dist_px;
        for (size_t i = 0; i + 1 < track.blobs.size(); ++i) {
            double d = pointSegDist(cone.cx, cone.cy,
                                    track.blobs[i].cx, track.blobs[i].cy,
                                    track.blobs[i + 1].cx, track.blobs[i + 1].cy);
            if (d <= thr) return true;
        }
        return std::abs(cone.cx - track.mid_x) <= thr
            && std::abs(cone.cy - track.mid_y) <= thr * 1.5;
    }

    bool pickBlueOnTrack(const std::vector<Blob>& blues, const Track* track, Blob& out) const
    {
        if (!track) return false;
        std::vector<Blob> on;
        for (const auto& b : blues) {
            if (b.area >= cfg_.blue_cone_min_area && coneOnTrack(b, *track)) on.push_back(b);
        }
        return pickNearest(on, cfg_.blue_cone_min_area, out);
    }

    // ★ 通用"挡路锥桶"判定：红锥、蓝锥共用。
    // 阈值随锥桶远近自动放大（cy 越大=越近，允许的横向偏差越大），
    // 这样赛道边缘的蓝锥也能被正确识别为挡路，而不是要求它贴着中间黑点连线。
    bool pickConeInPath(const std::vector<Blob>& cones, double min_area,
                        const Track* track, Blob& out) const
    {
        double ref = track ? track->mid_x : CENTER_X;
        std::vector<Blob> on;
        for (const auto& b : cones) {
            if (b.area < min_area) continue;
            double thr = CONE_PATH_HALF_W + 90.0 * (b.cy / static_cast<double>(IMG_H));
            if (std::abs(b.cx - ref) <= thr) on.push_back(b);
        }
        return pickNearest(on, min_area, out);
    }

    bool gotoLocal(double tx, double ty, bool face_x)
    {
        double dx = tx - x_, dy = ty - y_;
        double dist = std::hypot(dx, dy);
        double bearing = std::atan2(dy, dx);
        double yaw_err = wrapPi(bearing - yaw_);
        if (dist > cfg_.goal_pos_tol) {
            double vx = 0, wz = clampd(cfg_.yaw_p * yaw_err, -cfg_.vel_z_max, cfg_.vel_z_max);
            if (std::abs(yaw_err) <= SPIN_YAW)
                vx = clampd(cfg_.pos_p * dist, 0.1125, cfg_.vel_x_odom);   // 爬行下限 ×1.5（再乘 1.5）
            send(vx, wz);
            erro_ = yaw_err;
            return false;
        }
        if (face_x) {
            yaw_err = wrapPi(0.0 - yaw_);
            if (std::abs(yaw_err) > cfg_.goal_yaw_tol) {
                send(0, clampd(cfg_.yaw_p * yaw_err, -cfg_.vel_z_max, cfg_.vel_z_max));
                erro_ = yaw_err;
                return false;
            }
        }
        sendStop();
        erro_ = 0;
        return true;
    }

    // 说明：避障已改为 navGoal() 内的「有界航向偏置」(avoid_yaw_)，
    // 方向由赛道路线决定（红卡→左、蓝卡→右），不再使用本函数。
    // 保留仅用于旧调用兼容，实际未被调用。
    void avoidCone(const Blob& cone, const std::string& color, double bias_wz = 0.0)
    {
        double wz_mag = (color == "blue") ? std::abs(cfg_.blue_avoid_wz) : std::abs(cfg_.red_avoid_wz);
        double vx = (color == "blue") ? cfg_.blue_avoid_vx : cfg_.red_avoid_vx;
        double wz = ((card_color_ == "blue") ? -wz_mag : wz_mag) + bias_wz;
        avoid_side_ = (card_color_ == "blue") ? "RIGHT" : "LEFT";
        avoid_color_ = color;
        avoid_blob_ = cone;
        avoiding_ = true;
        send(vx, clampd(wz, -cfg_.vel_z_max, cfg_.vel_z_max));
        erro_ = wz;
    }

    bool navGoal(const Track* track, const Blob* avoid_red, const Blob* avoid_blue)
    {
        avoiding_ = false;
        double dx = goal_x_ - x_, dy = goal_y_ - y_;
        double dist = std::hypot(dx, dy);
        double yaw_to_x = wrapPi(0.0 - yaw_);   // ★ 车头对准 +x 的误差（营地入口方向）
        // ★ 到点判定：位置进圈 + 车头对准 +x 才算完成；否则原地转正（有超时兜底）
        if (dist < cfg_.goal_pos_tol) {
            if (std::abs(yaw_to_x) < cfg_.goal_yaw_tol) {
                sendStop();
                erro_ = 0;
                goal_aligning_ = false;
                return true;
            }
            if (!goal_aligning_) {
                goal_aligning_ = true;
                goal_align_t0_ = std::chrono::steady_clock::now();
            }
            double waited = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - goal_align_t0_).count();
            if (waited > GOAL_ALIGN_MAX_SEC) {   // 摆不正也不再拖，兜底停车
                sendStop();
                erro_ = 0;
                goal_aligning_ = false;
                return true;
            }
            send(0.0, clampd(cfg_.yaw_p * yaw_to_x, -cfg_.vel_z_max, cfg_.vel_z_max));
            erro_ = yaw_to_x;
            return false;
        }
        goal_aligning_ = false;
        double yaw_err = wrapPi(std::atan2(dy, dx) - yaw_);

        // ★ 走廊约束：相对 base→goal 直线的有符号横向偏差（>0 表示偏左）
        double ux = goal_x_ - base_x_, uy = goal_y_ - base_y_;
        double un = std::hypot(ux, uy);
        if (un < 1e-6) un = 1.0;
        ux /= un; uy /= un;
        double cross = -(x_ - base_x_) * uy + (y_ - base_y_) * ux;
        double corridor_wz = 0.0;
        double over = std::abs(cross) - CORRIDOR_HALF_W;
        if (over > 0)
            corridor_wz = -std::copysign(CORRIDOR_GAIN * over, cross);

        // ★ 避让：方向由赛道路线决定（红卡→左，蓝卡→右），航向偏置有界（≤AVOID_YAW_MAX），不会绕圈
        const bool cone_here = (avoid_red != nullptr) || (avoid_blue != nullptr);
        const double off_dir = (card_color_ == "blue") ? -1.0 : 1.0;
        if (cone_here) {
            // 锥桶已经贴脸：不要停车（会卡死），直接满偏航向快速让开
            double area  = avoid_red ? avoid_red->area : avoid_blue->area;
            double max_a = avoid_red ? cfg_.red_cone_max_area : cfg_.blue_cone_max_area;
            if (area > max_a * 0.6) avoid_yaw_ = off_dir * AVOID_YAW_MAX;
            avoid_yaw_ += off_dir * AVOID_YAW_RATE;
            ++avoid_frames_;
            avoiding_    = true;
            avoid_color_ = avoid_red ? "red" : "blue";
            avoid_side_  = (off_dir > 0.0) ? "LEFT" : "RIGHT";
        } else {
            avoid_frames_ = 0;
            if (std::abs(avoid_yaw_) <= AVOID_YAW_RECOVER) avoid_yaw_ = 0;
            else avoid_yaw_ -= std::copysign(AVOID_YAW_RECOVER, avoid_yaw_);
        }
        avoid_yaw_ = clampd(avoid_yaw_, -AVOID_YAW_MAX, AVOID_YAW_MAX);
        const double avoid_wz = clampd(avoid_yaw_ * 2.0, -cfg_.vel_z_max, cfg_.vel_z_max);
        if (dist < cfg_.near_goal_dist) {
            // ★ 入口收敛：越接近终点，越把航向往 +x 拉，形成贴图那样的“平着进营地”
            double k = clampd(1.0 - dist / ENTRY_ALIGN_DIST, 0.0, 1.0) * ENTRY_ALIGN_W;
            double yaw_cmd = (1.0 - k) * yaw_err + k * yaw_to_x;
            double vx = clampd(NEAR_VX_P * dist, 0.1125, NEAR_VX_MAX);   // 爬行下限 ×1.5（再乘 1.5）
            send(vx, clampd(cfg_.yaw_p * yaw_cmd + corridor_wz + avoid_wz, -cfg_.vel_z_max, cfg_.vel_z_max));
            erro_ = yaw_cmd;
            return false;
        }
        if (track) {
            double vis_err = track->mid_x - CENTER_X;
            double wz = -vis_err * cfg_.vis_wz_gain + cfg_.odom_blend * cfg_.yaw_p * yaw_err
                      + corridor_wz + avoid_wz;
            double vx = cone_here ? std::min(cfg_.vel_x_vision, 0.27) : cfg_.vel_x_vision;   // 遇锥限速 ×1.5（再乘 1.5）
            send(vx, clampd(wz, -cfg_.vel_z_max, cfg_.vel_z_max));
            erro_ = vis_err / CENTER_X;
            return false;
        }
        double vx = std::abs(yaw_err) > LOST_YAW ? 0.0 : LOST_VX;
        if (cone_here) vx = std::min(vx, 0.18);   // 丢线遇锥限速 ×1.5（再乘 1.5）
        send(vx, clampd(cfg_.yaw_p * yaw_err + corridor_wz + avoid_wz, -cfg_.vel_z_max, cfg_.vel_z_max));
        erro_ = yaw_err;
        return false;
    }

    void acceptCard(const std::string& color)
    {
        if (phase_ != Phase::WaitCard) return;
        card_color_ = color;
        if (color == "blue") {
            base_x_ = cfg_.blue_base_x; base_y_ = cfg_.blue_base_y;
            goal_x_ = cfg_.blue_goal_x; goal_y_ = cfg_.blue_goal_y;
        } else {
            base_x_ = cfg_.red_base_x; base_y_ = cfg_.red_base_y;
            goal_x_ = cfg_.red_goal_x; goal_y_ = cfg_.red_goal_y;
        }
        x_ = y_ = yaw_ = 0;
        phase_ = Phase::NavBase;
        std::cout << "\nCARD=" << color
                  << " base(" << base_x_ << "," << base_y_ << ")"
                  << " goal(" << goal_x_ << "," << goal_y_ << ")"
                  << " gyro_bias=" << gyro_bias_ << " rad/s n=" << bias_n_ << "\n";
    }

    void update(const cv::Mat& hsv, const Track* track,
                const Blob* avoid_red, const Blob* avoid_blue,
                cv::Rect& card_box, bool& has_card)
    {
        has_card = false;
        erro_ = 0;
        if (phase_ == Phase::WaitCard) {
            avoiding_ = false;
            sendStop();
            CardStat b = cardStats(hsv, "blue");
            CardStat r = cardStats(hsv, "red");
// 【判定点 1】色卡成立的充要条件：面积占比 ≥ card_fill_ratio，且是另一色的 card_dominance 倍
            bool blue_ok = b.ratio >= cfg_.card_fill_ratio && b.ratio >= r.ratio * cfg_.card_dominance;
            bool red_ok = r.ratio >= cfg_.card_fill_ratio && r.ratio >= b.ratio * cfg_.card_dominance;
            if (blue_ok && !red_ok) {
                ++hold_blue_;
                hold_red_ = 0;
                card_label_ = "BLUE";
            } else if (red_ok && !blue_ok) {
                ++hold_red_;
                hold_blue_ = 0;
                card_label_ = "RED";
            } else {
                hold_blue_ = hold_red_ = 0;
                card_label_ = "waiting";
            }
            if (b.fill >= r.fill && b.has_box) { card_box = b.box; has_card = true; }
            else if (r.has_box) { card_box = r.box; has_card = true; }
// 【判定点 2】必须连续 card_confirm_frames 帧成立才真正接受色卡 → 进入 NAV_BASE 起步
            if (hold_blue_ >= cfg_.card_confirm_frames) acceptCard("blue");
            else if (hold_red_ >= cfg_.card_confirm_frames) acceptCard("red");
            return;
        }
        card_label_.clear();
        if (!have_odom_) { sendStop(); return; }

        if (phase_ == Phase::NavBase) {
            if (gotoLocal(base_x_, base_y_, true)) {
                phase_ = Phase::WaitBase;
                wait_t0_ = std::chrono::steady_clock::now();
                sendStop();
                std::cout << "\n到达据点，停车 " << cfg_.base_wait_sec << " 秒\n";
            }
            return;
        }
        if (phase_ == Phase::WaitBase) {
            sendStop();
            double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_t0_).count();
            if (sec >= cfg_.base_wait_sec) {
                phase_ = Phase::NavGoal;
                std::cout << "\n离开据点，前往终点\n";
            }
            return;
        }
        if (phase_ == Phase::NavGoal) {
            if (navGoal(track, avoid_red, avoid_blue)) {
                phase_ = Phase::Done;
                sendStop();
                std::cout << "\n到达终点，停车\n";
            }
            return;
        }
        avoiding_ = false;
        sendStop();
    }

    void drawBlob(cv::Mat& img, const Blob& b, const cv::Scalar& color, const std::string& label, int thick) const
    {
        int x1 = static_cast<int>(b.cx - b.w * 0.5);
        int y1 = static_cast<int>(b.cy - b.h * 0.5);
        int x2 = static_cast<int>(b.cx + b.w * 0.5);
        int y2 = static_cast<int>(b.cy + b.h * 0.5);
        cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), color, thick);
        cv::circle(img, cv::Point(static_cast<int>(b.cx), static_cast<int>(b.cy)), 3, color, -1);
        if (!label.empty())
            cv::putText(img, label, cv::Point(x1, std::max(14, y1 - 4)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
    }

    void draw(cv::Mat& image, const std::vector<Blob>& blacks,
              const std::vector<Blob>& reds, const std::vector<Blob>& blues,
              const Track* track, const cv::Rect* card_box) const
    {
        cv::Mat overlay = image.clone();
        cv::rectangle(overlay, cv::Point(0, 0), cv::Point(IMG_W - 1, std::max(0, roi_y_ - 1)),
                      cv::Scalar(40, 40, 40), -1);
        cv::addWeighted(overlay, 0.25, image, 0.75, 0, image);
        if (roi_y_ > 0 && roi_y_ < IMG_H)
            cv::line(image, cv::Point(0, roi_y_), cv::Point(IMG_W - 1, roi_y_), cv::Scalar(0, 255, 255), 1);
        cv::line(image, cv::Point(static_cast<int>(CENTER_X), roi_y_),
                 cv::Point(static_cast<int>(CENTER_X), IMG_H), cv::Scalar(80, 80, 80), 1);

        if (card_box && phase_ == Phase::WaitCard) {
            cv::rectangle(image, *card_box, cv::Scalar(255, 255, 0), 2);
            cv::putText(image, "CARD?", cv::Point(card_box->x, std::max(16, card_box->y - 4)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        }
        for (const auto& b : blacks) drawBlob(image, b, cv::Scalar(160, 160, 160), "", 1);
        if (!avoiding_) {
            for (const auto& b : reds) drawBlob(image, b, cv::Scalar(0, 80, 255), "R", 1);
            for (const auto& b : blues) drawBlob(image, b, cv::Scalar(255, 140, 0), "B", 1);
        }
        if (track) {
            for (size_t i = 0; i < track->blobs.size(); ++i) {
                drawBlob(image, track->blobs[i], cv::Scalar(0, 255, 0), "K", 2);
                if (i + 1 < track->blobs.size()) {
                    cv::line(image,
                             cv::Point(static_cast<int>(track->blobs[i].cx), static_cast<int>(track->blobs[i].cy)),
                             cv::Point(static_cast<int>(track->blobs[i + 1].cx), static_cast<int>(track->blobs[i + 1].cy)),
                             cv::Scalar(0, 255, 255), 2);
                }
            }
            cv::circle(image, cv::Point(static_cast<int>(track->mid_x), static_cast<int>(track->mid_y)),
                       6, cv::Scalar(0, 255, 0), -1);
        }
        if (avoiding_) {
            cv::Scalar edge = avoid_color_ == "blue" ? cv::Scalar(255, 200, 0) : cv::Scalar(0, 255, 255);
            drawBlob(image, avoid_blob_, edge, std::string("AVOID ") + avoid_side_, 2);
        }

        std::ostringstream l1, l2;
        l1 << phaseName(phase_) << " " << (card_color_.empty() ? "-" : card_color_)
           << (motion_enabled_ ? " RUN" : " STOP");
        l2 << "x=" << x_ << " y=" << y_ << " yaw=" << (yaw_ * 180.0 / kPi);
        cv::putText(image, l1.str(), cv::Point(4, 14), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        cv::putText(image, l2.str(), cv::Point(4, 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        if (!card_label_.empty())
            cv::putText(image, card_label_, cv::Point(4, 48), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
        if (!motion_enabled_)
            cv::putText(image, "MOTION STOPPED", cv::Point(IMG_W / 2 - 70, 80),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    }

    void printHsv() const
    {
        auto p = [](const char* n, const std::array<int, 6>& h) {
            std::cout << n << " = (" << h[0] << ", " << h[1] << ", " << h[2]
                      << ", " << h[3] << ", " << h[4] << ", " << h[5] << ")\n";
        };
        p("HUE_BLACK", hue_black_);
        p("HUE_RED_1", hue_red_1_);
        p("HUE_RED_2", hue_red_2_);
        p("HUE_BLUE", hue_blue_);
    }

    bool loadHsv(bool verbose)
    {
        std::ifstream in(cfg_.hsv_file);
        if (!in) {
            if (verbose) std::cout << "没有 HSV 文件: " << cfg_.hsv_file << "\n";
            return false;
        }
        std::string line;
        while (std::getline(in, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            auto eq = line.find('=');
            auto lp = line.find('(');
            auto rp = line.find(')');
            if (eq == std::string::npos || lp == std::string::npos || rp == std::string::npos) continue;
            std::string key = trim(line.substr(0, eq));
            std::array<int, 6> hue{};
            if (!parseHueBody(line.substr(lp + 1, rp - lp - 1), hue)) continue;
            if (key == "HUE_BLACK") hue_black_ = hue;
            else if (key == "HUE_RED_1") hue_red_1_ = hue;
            else if (key == "HUE_RED_2") hue_red_2_ = hue;
            else if (key == "HUE_BLUE") hue_blue_ = hue;
        }
        if (verbose) {
            std::cout << "已重新加载 " << cfg_.hsv_file << "\n";
            printHsv();
        }
        return true;
    }

    void saveHsv() const
    {
        std::ofstream out(cfg_.hsv_file);
        if (!out) {
            std::cerr << "无法写入 " << cfg_.hsv_file << "\n";
            return;
        }
        out << "# beacon_race HSV\n";
        auto w = [&](const char* n, const std::array<int, 6>& h) {
            out << n << " = (" << h[0] << ", " << h[1] << ", " << h[2]
                << ", " << h[3] << ", " << h[4] << ", " << h[5] << ")\n";
        };
        w("HUE_BLACK", hue_black_);
        w("HUE_RED_1", hue_red_1_);
        w("HUE_RED_2", hue_red_2_);
        w("HUE_BLUE", hue_blue_);
        std::cout << "HSV 已保存 -> " << cfg_.hsv_file << "\n";
    }

    void editHsv()
    {
        sendStop();
        keyboardRestore();
        std::cout << "\nHSV 编辑（车已停车）。示例: blue 100 80 46 124 255 255\n"
                  << "颜色名: black red1 red2 blue ，空行返回，q 保存并返回\n";
        printHsv();
        std::string line;
        while (g_running && std::getline(std::cin, line)) {
            line = trim(line);
            if (line.empty()) break;
            if (line == "q" || line == "Q") { saveHsv(); break; }
            std::istringstream ss(line);
            std::string name;
            std::array<int, 6> hue{};
            ss >> name;
            bool ok = true;
            for (int i = 0; i < 6; ++i) if (!(ss >> hue[i])) ok = false;
            if (!ok) {
                std::cout << "需要 6 个整数\n";
                continue;
            }
            clampHue(hue);
            if (name == "black") hue_black_ = hue;
            else if (name == "red1") hue_red_1_ = hue;
            else if (name == "red2") hue_red_2_ = hue;
            else if (name == "blue") hue_blue_ = hue;
            else {
                std::cout << "未知颜色，用 black/red1/red2/blue\n";
                continue;
            }
            printHsv();
        }
        keyboardInit();
    }

    void handleKeys()
    {
        int key = getKey();
        if (key < 0) return;
        char c = static_cast<char>(key);
        if (c == 'c' || c == 'C' || c == '\x03') {
            sendStop();
            g_running = false;
        } else if (c == 'x' || c == 'X') {
            motion_enabled_ = !motion_enabled_;
            if (!motion_enabled_) sendStop();
            std::cout << (motion_enabled_ ? "\n运动恢复\n" : "\n运动停止\n");
        } else if (c == 'q' || c == 'Q') {
            saveHsv();
        } else if (c == 'l' || c == 'L') {
            loadHsv(true);
        } else if (c == 'f' || c == 'F') {
            editHsv();
        } else if (c == 'v' || c == 'V') {
            view_ = (view_ + 1) % 4;
            const char* names[] = {"debug", "black", "red", "blue"};
            std::cout << "\n画面: " << names[view_] << "\n";
        } else if (c == 'b' || c == 'B' || c == 'r' || c == 'R') {
            if (!cfg_.allow_force_card)
                std::cout << "\n未允许强制选色，配置 allow_force_card=true 或加 --force-card\n";
            else
                acceptCard((c == 'b' || c == 'B') ? "blue" : "red");
        }
    }
};

// ┌────────────────────────────────────────────────────────────────────────┐
// │ 命令行参数一览（优先级高于 cfg 文件）：                                │
// │   --config FILE   指定配置文件                                        │
// │   --uart DEV      串口，默认 /dev/ttyUSB0（板子）/ /dev/deepbotbase（ROS）│
// │   --cam N         摄像头编号（多路 USB 摄像头时用）                    │
// │   --no-drive      只跑视觉、不发速度（调视觉必用）                     │
// │   --force-card    允许 b/r 手动选色（比赛别开）                        │
// │   --no-stream     关闭 9999 图传（省 CPU）                             │
// └────────────────────────────────────────────────────────────────────────┘
static void printHelp()
{
    std::cout
        << "beaconRace [--config 文件] [--uart /dev/ttyUSB0] [--cam 0]\n"
        << "           [--no-drive] [--force-card] [--no-stream]\n"
        << "默认同目录或当前目录的 beacon_race.cfg。底盘协议与 lineFollow 相同。\n";
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    Config cfg;
    std::string config_arg;
    bool no_drive = false, force_card = false, no_stream = false, have_uart = false;
    int cam = -1;
    std::string uart;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << name << " 缺少参数\n";
                std::exit(1);
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") { printHelp(); return 0; }
        else if (a == "--config") config_arg = need("--config");
        else if (a == "--uart") { uart = need("--uart"); have_uart = true; }
        else if (a == "--cam") cam = std::atoi(need("--cam"));
        else if (a == "--no-drive") no_drive = true;
        else if (a == "--force-card") force_card = true;
        else if (a == "--no-stream") no_stream = true;
        else { std::cerr << "未知参数: " << a << "\n"; printHelp(); return 1; }
    }

// ★★★ 【关键陷阱】配置加载顺序：后加载的覆盖先加载的 ★★★
//     第 1048 行：程序所在目录 /beacon_race.cfg   （exeDir()）
//     第 1049 行：当前工作目录 /beacon_race.cfg   ← 会覆盖上一条
//     第 1050 行：--config 指定的文件            ← 优先级最高
//   「改了 cfg 没反应」十有八九是程序所在目录那份把它盖住了。
//   排查：在板子上分别 ls -l /home/root/beacon_race.cfg 和当前目录下的同名文件。
    loadConfigFile(exeDir() + "/beacon_race.cfg", cfg, false);
    loadConfigFile("beacon_race.cfg", cfg, false);
    if (!config_arg.empty() && !loadConfigFile(config_arg, cfg, true)) return 1;
// 【命令行覆盖】--no-drive / --force-card / --no-stream / --cam / --uart 在这里写入 cfg，
// 所以它们总是能压过文件和默认值。
    if (no_drive) cfg.enable_drive = false;
    if (force_card) cfg.allow_force_card = true;
    if (no_stream) cfg.stream_enable = false;
    if (cam >= 0) cfg.camera_id = cam;
    if (have_uart) cfg.uart = uart;

    SerialPort port;
    if (!port.openPort(cfg.uart, 115200)) {
        std::cerr << "打开串口失败: " << cfg.uart
                  << " （久久派示例用 /dev/ttyUSB0，ROS 驱动常用 /dev/deepbotbase）\n";
        return 1;
    }
    std::thread reader(readThreadFunc, &port, std::ref(g_running));
    BeaconRace race(&port, cfg);
    int rc = race.run();
    g_running = false;
    sendVelocity(&port, 0, 0, 0);
    if (reader.joinable()) reader.join();
    port.closePort();
    return rc;
}

