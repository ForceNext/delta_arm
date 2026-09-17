#include <iostream>
#include <vector>
#include <cmath>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <cerrno>

#include "modbus_master.hpp"
#include "yaml-cpp/yaml.h"

// ---------------------------------------------------------------------------
// 正点原子 PDxxS1 命令码：驱动把「寄存器起始地址」当作命令码用（高字节补 0），
// 读写都走标准 Modbus 帧（读=0x03/0x04，写=0x06/0x10）。
// ---------------------------------------------------------------------------
namespace Cmd {
    constexpr uint16_t READ_POS     = 0x002A;  // 读实时位置（int32，51200=一圈）
    constexpr uint16_t READ_STATE   = 0x002C;  // 读运行状态
    constexpr uint16_t ABS_POS_OPEN = 0x00E1;  // 开环绝对位置：方向(1)+加减速(1)+速度(2)+位置(4)
    constexpr uint16_t ABS_POS      = 0x00F2;  // 绝对位置模式（闭环），字段顺序需对照手册确认
    constexpr uint16_t ENABLE       = 0x00FA;  // 使能：0=使能 / 1=失能（参数按手册确认）
    constexpr uint16_t STOP         = 0x00FC;  // 立即停止（刹车）
}

constexpr double PI = 3.14159265358979323846;

// 3 台电机从站地址（来自 configs/hardware_config.yaml 的 motors[].addr，解析失败用默认）
static std::vector<uint8_t> g_addr = {1, 2, 3};
static uint64_t g_err[3] = {0, 0, 0};          // 每台累计失败次数
static std::atomic<bool> g_running{true};

static void onSigInt(int) { g_running = false; }

// 从 yaml 读电机地址
static void loadMotorAddrs(const std::string& path)
{
    try {
        YAML::Node root = YAML::LoadFile(path);
        std::vector<uint8_t> a;
        for (const auto& m : root["motors"])
            a.push_back((uint8_t)m["addr"].as<int>());
        if (a.size() >= 3) g_addr = a;
    } catch (const std::exception& e) {
        std::cerr << "读取电机地址失败(" << e.what() << ")，使用默认 {1,2,3}\n";
    }
}

// 绝对时间睡眠到 next（CLOCK_MONOTONIC + TIMER_ABSTIME，避免周期漂移）
static void sleepUntil(const std::chrono::steady_clock::time_point& next)
{
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  next.time_since_epoch()).count();
    struct timespec ts{ns / 1000000000LL, ns % 1000000000LL};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, onSigInt);

    SerialPort port(argc, argv);          // 打开串口（读 hardware_config.yaml）
    ModbusMaster master(port);

    const std::string cfg = (argc > 1) ? argv[1] : "configs/hardware_config.yaml";
    loadMotorAddrs(cfg);

    // 可选：先使能 3 台电机（参数按手册确认，0=使能）
    // for (int i = 0; i < 3; ++i) master.writeRegister(g_addr[i], Cmd::ENABLE, 0x0000);

    const double freq = 200.0;            // 控制频率 200Hz
    const auto period = std::chrono::nanoseconds((long long)(1e9 / freq));

    // 演示轨迹参数（占位：这里应替换成正逆解/轨迹规划结果）
    const double center[3] = {5120.0, 5120.0, 5120.0};  // 偏移到非负区间（51200=一圈）
    const double amp[3]    = {5120.0, 5120.0, 5120.0};  // 位置幅度（±0.1 圈）
    const double omega[3]  = {1.0, 0.5, 0.8};           // 角频率 rad/s

    std::cout << "开始 200Hz 控制循环（Ctrl-C 退出）: 电机地址 "
              << (int)g_addr[0] << ',' << (int)g_addr[1] << ',' << (int)g_addr[2] << "\n";

    auto next = std::chrono::steady_clock::now();
    uint64_t cycle = 0;

    while (g_running) {
        // 1) 计算 3 台目标位置并逐台下发（失败跳过，不阻塞周期）
        for (int i = 0; i < 3; ++i) {
            double t = (double)cycle / freq;
            int32_t pos = (int32_t)std::lround(center[i] + amp[i] * std::sin(2.0 * PI * omega[i] * t));

            // 2) 组 0xE1 开环绝对位置命令：方向(1)+加减速(1)+速度(2)+位置(4) = 8 字节 = 4 寄存器
            uint16_t regs[4];
            regs[0] = 0x0000;                            // 方向 0=正转 | 加减速 0=直接启动
            regs[1] = 100;                               // 速度 100 RPM
            regs[2] = (uint16_t)((uint32_t)pos >> 16);   // 位置高 16 位
            regs[3] = (uint16_t)((uint32_t)pos & 0xFFFF);// 位置低 16 位

            if (!master.writeRegisters(g_addr[i], Cmd::ABS_POS_OPEN, 4, regs))
                ++g_err[i];
        }

        // 3) 周期统计
        if (++cycle % 1000 == 0) {
            std::cout << "[cycle " << cycle << "] err="
                      << g_err[0] << '/' << g_err[1] << '/' << g_err[2] << "\n";
        }

        // 4) 睡到下一个绝对周期边界
        auto now = std::chrono::steady_clock::now();
        next += period;
        if (next < now) next = now + period;             // 发生过载，跳到下一拍
        sleepUntil(next);
    }

    std::cout << "退出：停止 3 台电机\n";
    for (int i = 0; i < 3; ++i)
        master.writeRegister(g_addr[i], Cmd::STOP, 0x0001);
    return 0;
}
