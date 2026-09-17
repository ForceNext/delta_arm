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
#include "shared_memory.h"
#include "yaml-cpp/yaml.h"

// ---------------------------------------------------------------------------
// 正点原子 PDxxS1 命令码：驱动把「寄存器起始地址」当作命令码用（高字节补 0），
// 读写都走标准 Modbus 帧（读=0x03/0x04，写=0x06/0x10）。
// ---------------------------------------------------------------------------
namespace Cmd {
    constexpr uint16_t READ_POS   = 0x002A;  // 读实时位置（int32，51200=一圈），也用于在线探测
    constexpr uint16_t READ_SPEED = 0x0029;  // 读实时转速（int16，单位 RPM）
    constexpr uint16_t ABS_POS    = 0x00F2;  // 闭环绝对位置模式：方向(1)+加减速(1)+速度(2)+位置(4)
    constexpr uint16_t ENABLE     = 0x00FA;  // 使能：0=使能 / 1=失能
}

// 运动参数（来自上位机实测）：加减速 200，速度 1000 RPM
constexpr uint8_t  MOVE_ACCEL = 200;
constexpr uint16_t MOVE_SPEED = 1000;

// 角度（度）→ 电机计数：51200 = 一圈 360°
constexpr double DEG2CNT = 51200.0 / 360.0;

static std::vector<uint8_t> g_cfg_addr = {1, 2, 3};  // 配置里的所有从站地址
static std::vector<uint8_t> g_addr;                  // 实际在线、要控制的从站地址
static std::vector<uint64_t> g_err;                  // 每台累计失败次数
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
        if (!a.empty()) g_cfg_addr = a;
    } catch (const std::exception& e) {
        std::cerr << "读取电机地址失败(" << e.what() << ")，使用默认 {1,2,3}\n";
    }
}

// 探测在线电机：逐个读实时位置（0x2A），能应答的才纳入控制列表
static void detectMotors(ModbusMaster& master)
{
    g_addr.clear();
    for (uint8_t a : g_cfg_addr) {
        uint16_t buf[2] = {0, 0};
        if (master.readInputRegisters(a, Cmd::READ_POS, 2, buf))
            g_addr.push_back(a);
    }
    if (g_addr.empty()) {
        std::cerr << "未探测到任何在线电机，退回配置地址\n";
        g_addr = g_cfg_addr;
    }
    g_err.assign(g_addr.size(), 0);
    std::cout << "探测到 " << g_addr.size() << " 台在线电机\n";
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
    std::cout << std::unitbuf;

    SerialPort port(argc, argv);          // 打开串口（读 hardware_config.yaml）
    ModbusMaster master(port);
    master.resp_timeout_ms = 20;

    const std::string cfg = (argc > 1) ? argv[1] : "configs/hardware_config.yaml";
    loadMotorAddrs(cfg);
    detectMotors(master);

    SharedMemory shm;                     // 共享内存（默认名称 / 信号量 key）

    // 启动时使能所有在线电机（0=使能）
    for (uint8_t a : g_addr)
        master.writeRegister(a, Cmd::ENABLE, 0x0000);

    const double freq = 200.0;            // 控制频率 200Hz
    const auto period = std::chrono::nanoseconds((long long)(1e9 / freq));

    std::cout << "开始 200Hz 控制循环（读共享内存 xyz 作为三个目标角度，单位：度）\n";

    auto start = std::chrono::steady_clock::now();
    auto next  = start;
    uint64_t cycle = 0;

    while (g_running) {
        // 1) 读上位机命令：xyz 直接作为三个电机的目标角度（mode=1 时生效）
        ArmCommand cmd;
        shm.readCommand(cmd);

        // 2) 角度 → 电机计数，逐台下发
        double target[3] = {cmd.x, cmd.y, cmd.z};
        uint32_t counts[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i)
            counts[i] = (uint32_t)llround(target[i] * DEG2CNT);

        if (cmd.mode == 1) {
            for (size_t i = 0; i < g_addr.size() && i < 3; ++i) {
                uint16_t regs[4];
                regs[0] = (uint16_t)((0x00 << 8) | MOVE_ACCEL);  // 方向 0=正转 | 加减速
                regs[1] = MOVE_SPEED;                            // 速度 RPM
                regs[2] = (uint16_t)(counts[i] >> 16);
                regs[3] = (uint16_t)(counts[i] & 0xFFFF);

                if (!master.writeRegisters(g_addr[i], Cmd::ABS_POS, 4, regs))
                    ++g_err[i];
            }
        }

        // 3) 发布状态（简单版：在线 + 目标位置 + 回显坐标）
        {
            ArmStatus st{};
            st.seq = cycle;
            st.state = (cmd.mode == 1) ? 1 : 0;
            for (size_t i = 0; i < g_addr.size() && i < 3; ++i) {
                st.online[i] = 1;
                st.theta[i] = target[i];
                st.motor_pos[i] = (double)counts[i];
            }
            st.x = cmd.x; st.y = cmd.y; st.z = cmd.z;
            shm.writeStatus(st);
        }

        ++cycle;

        // 4) 周期统计
        if (cycle % 200 == 0) {
            double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            std::cout << "[cycle " << cycle << "] 实际频率 " << (cycle / elapsed) << " Hz";
            for (size_t i = 0; i < g_addr.size(); ++i)
                std::cout << "  电机" << (int)g_addr[i] << "错误=" << g_err[i];
            std::cout << "\n";
        }

        // 5) 睡到下一个绝对周期边界
        auto now = std::chrono::steady_clock::now();
        next += period;
        if (next < now) next = now + period;
        sleepUntil(next);
    }

    std::cout << "退出（不刹车，电机保持当前位置）\n";
    return 0;
}
