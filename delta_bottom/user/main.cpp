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
#include "MotorMapping.hpp"
#include "Utilities/SharedMemory.h"

// 运动参数（来自上位机实测）：加减速 200，速度 1000 RPM
constexpr uint8_t  MOVE_ACCEL = 200;
constexpr uint16_t MOVE_SPEED = 1000;

// 角度（度）→ 电机计数：51200 = 一圈 360°
constexpr double DEG2CNT = 51200.0 / 360.0;

static std::atomic<bool> g_running{true};

static void onSigInt(int) { g_running = false; }

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
    MotorMapping::loadMotorAddrs(cfg);
    MotorMapping::detectMotors(master);

    // 探测结果以引用取出，后续控制循环沿用 g_addr / g_err 变量名
    const auto& g_addr = MotorMapping::addrs();
    auto& g_err = MotorMapping::errs();

    ArmSharedMemory shm;                  // 共享内存（命令/状态 + 信号量互斥）
    shm.connect();                        // 连接：首次创建，其余附加

    // 启动时使能所有在线电机
    MotorMapping::enableMotors(master);

    const double freq = 200.0;            // 控制频率 200Hz
    const auto period = std::chrono::nanoseconds((long long)(1e9 / freq));

    std::cout << "开始 200Hz 控制循环（读共享内存 xyz 作为三个目标角度，单位：度）\n";

    auto start = std::chrono::steady_clock::now();
    auto next  = start;
    uint64_t cycle = 0;

    while (g_running) 
    {
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
                // 下发绝对位置：方向 0=正转，加减速 MOVE_ACCEL，速度 MOVE_SPEED
                if (!MotorMapping::moveAbsolute(master, g_addr[i], counts[i],
                                                0, MOVE_ACCEL, MOVE_SPEED))
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
