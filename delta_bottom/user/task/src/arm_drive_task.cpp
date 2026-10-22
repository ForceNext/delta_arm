#include "arm_drive_task.hpp"
#include <cmath>

// —— 下发参数 ——
constexpr uint8_t  MOVE_ACCEL = 200;    // 加减速 r/s²
constexpr uint16_t MOVE_SPEED = 1000;   // 速度 RPM

// 共享内存里的 theta 是什么角度：
//   true  = 【关节角】，计数 = theta × 减速比 × 51200/360       ← 当前（下发时乘 joint.gear_ratio）
//   false = 【电机角】，计数 = theta × 51200/360
constexpr bool THETA_IS_JOINT_ANGLE = true;

// 等待上电时每 PROBE_PERIOD 拍（5ms × 20 = 100ms）探一台，避免把循环堵住
constexpr uint32_t PROBE_PERIOD = 20;

// 连续"无应答"多少次后，专门探一次该电机是否还在线（每台被下发间隔约 15ms → 约 150ms）
constexpr uint32_t TIMEOUT_BEFORE_PROBE = 10;

// 判定掉线后，每 OFFLINE_REPROBE_PERIOD 拍（5ms × 200 = 1s）轮转重探一台离线电机
constexpr uint32_t OFFLINE_REPROBE_PERIOD = 200;

ArmDriveTask::ArmDriveTask(PeriodicTaskManager* taskManager, float period, std::string name,
                           int argc, char** argv)
    : PeriodicTask(taskManager, period, name),
      port_(argc, argv),
      master_(port_),
      hw_cfg_((argc > 1) ? argv[1] : "configs/hardware_config.yaml")
{
    master_.resp_timeout_ms = 20;   // 等应答首字节
    // 帧内字节间隔：驱动器在电机运动/忙碌时回帧可能被拆成两段，
    // 5ms 会把中间的正常停顿误判成超时，放宽到 12ms。
    master_.byte_timeout_ms = 12;
    // 发帧前要求总线静默 1ms，避免上一帧的尾巴被下一次事务当成应答
    master_.idle_gap_ms = 1;

    int latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1) {
        printf("error: open /dev/cpu_dma_latency\n");
    } else {
        int latency_target_value = 0;
        int err = write(latency_target_fd, &latency_target_value, 4);
        if (err == -1) {
            printf("error: write /dev/cpu_dma_latency\n");
        } else {
            printf("set /dev/cpu_dma_latency %d\n", latency_target_value);
        }
    }
}

void ArmDriveTask::print_loop_rate()
{
    loop_counter_++;

    if (loop_counter_ == 1) {
        time_start_sec_ = time_ctrl.getSeconds();
        return;
    }
    if (loop_counter_ >= 1000) {
        double now = time_ctrl.getSeconds();
        double duration = now - time_start_sec_;
        double hz = 1000.0 / duration;

        printf_color(PrintColor::Green, "[DriveTask] 1000 loops: %.6f s  ->  %.2f Hz\n",
                     duration, hz);

        loop_counter_ = 0;
    }
}

void ArmDriveTask::init()
{
    // 周期统计
    loop_counter_ = 0;
    time_start_sec_ = 0.0;
    time_ctrl.start();
    cycle_ = 0;
    drives_ready_ = false;
    offline_probe_tick_ = 0;
    consec_timeout_[0] = consec_timeout_[1] = consec_timeout_[2] = 0;
    next_motor_ = 1;
    next_probe_ = 1;
    probe_tick_ = 0;
    last_rc_[0] = last_rc_[1] = last_rc_[2] = 0;

    // 上下位机共享内存（每次启动清零）
    shm_.connect();

    // 串口 + 电机地址 + 减速比
    MotorMapping::loadMotorAddrs(hw_cfg_);
    MotorMapping::loadGearRatios(hw_cfg_);
}

// 首次下发前把驱动器准备好：探测 → 初始化（协议 Modbus + 细分 16）→ 使能。
// 没使能/没切到通信位置模式时，位置指令不会让电机动作，所以这一步是必要的。
bool ArmDriveTask::ensureDrivesReady()
{
    if (++probe_tick_ < PROBE_PERIOD) return false;   // 每 100ms 只探一台
    probe_tick_ = 0;

    const int id = next_probe_;
    next_probe_ = (next_probe_ % 3) + 1;
    if (!MotorMapping::isOnline(id) && MotorMapping::probeMotorRc(master_, id) == MB_RC_OK)
        MotorMapping::setOnline(id, true);

    if (!MotorMapping::allOnline()) return false;

    for (int i = 1; i <= 3; ++i) {
        if (!MotorMapping::initMotor(master_, i)) {
            printf_color(PrintColor::Red, "[DriveTask] 电机 %d 初始化失败，稍后重试\n", i);
            return false;
        }
    }
    MotorMapping::enableMotors(master_);
    drives_ready_ = true;
    printf_color(PrintColor::Green, "[DriveTask] 三台在线、已初始化并使能，开始按共享内存角度下发\n");
    return true;
}

void ArmDriveTask::noteResult(int id, int rc)
{
    const int i = id - 1;

    if (rc != MB_RC_TIMEOUT) {
        // 有应答就算活着（哪怕 CRC 错、从站异常码：说明通信链路是通的）
        consec_timeout_[i] = 0;
        return;
    }

    if (++consec_timeout_[i] < TIMEOUT_BEFORE_PROBE) return;

    // 连续无应答达到阈值：专门探一次，确认是不是真掉线
    const int prc = MotorMapping::probeMotorRc(master_, id);
    const uint32_t n = consec_timeout_[i];
    consec_timeout_[i] = 0;

    if (prc == MB_RC_OK) {
        printf_color(PrintColor::Yellow,
                     "[DriveTask] 电机 %d 连续 %u 次超时，但单独探测有应答，仍按在线处理\n", id, n);
        return;
    }

    if (MotorMapping::isOnline(id)) {
        MotorMapping::setOnline(id, false);
        printf_color(PrintColor::Red,
                     "[DriveTask] 电机 %d 连续 %u 次超时且探测无应答（%s），判定掉线；"
                     "每秒重探，恢复后自动重新下发\n", id, n, modbusRcName(prc));
    }
}

void ArmDriveTask::publishStatus(const ArmCommand& cmd)
{
    ArmStatus st{};
    st.seq = cycle_++;                                   // 每拍递增：上位机据此判活
    // bit0 = 正在按命令下发；bit1（实测有效）暂不置位，因为还没有回读
    st.state = (uint8_t)(((cmd.mode == 1) && drives_ready_) ? 0x01 : 0x00);
    for (int i = 0; i < 3; ++i)
        st.online[i] = MotorMapping::isOnline(i + 1) ? 1 : 0;
    shm_.writeStatus(st);
}

void ArmDriveTask::run()
{
    print_loop_rate();

    // ① 读共享内存里的三个目标角度（以及 mode）
    ArmCommand cmd;
    shm_.readCommand(cmd);

    // ② 发布状态（seq 递增；工具脚本靠它判断底层是否在运行）
    publishStatus(cmd);

    // ③ 驱动器准备（只做一次）
    if (!drives_ready_ && !ensureDrivesReady()) return;

    // ④ 已判定掉线的电机：每 1s 轮转重探一台，通了就自动重新纳入下发
    if (++offline_probe_tick_ >= OFFLINE_REPROBE_PERIOD) {
        offline_probe_tick_ = 0;
        for (int id = 1; id <= 3; ++id) {
            if (MotorMapping::isOnline(id)) continue;          // 只探离线的
            if (MotorMapping::probeMotorRc(master_, id) == MB_RC_OK) {
                MotorMapping::setOnline(id, true);
                consec_timeout_[id - 1] = 0;
                last_rc_[id - 1] = 0;
                printf_color(PrintColor::Green,
                             "[DriveTask] 电机 %d 已恢复通信，重新纳入下发\n", id);
            }
            break;                                             // 每轮只探一台
        }
    }

    // ⑤ mode != 1 表示上位机空闲，不下发
    if (cmd.mode != 1) return;

    // ⑥ 每个周期只发一台，1→2→3 轮转（5ms 一拍 → 每台约 66Hz）
    const int id = next_motor_;
    next_motor_ = (next_motor_ % 3) + 1;
    if (!MotorMapping::isOnline(id)) return;

    // ⑦ 角度 → 电机计数。用 int32 中转再转 uint32，负角度按补码编码（驱动器位置是 int32）
    const double motor_deg = THETA_IS_JOINT_ANGLE
                                 ? cmd.theta[id - 1] * MotorMapping::gearRatio(id - 1)
                                 : cmd.theta[id - 1];
    const uint32_t counts =
        (uint32_t)(int32_t)std::llround(MotorMapping::angleToMotorPos(motor_deg));

    // ⑧ 下发绝对位置指令
    const int rc = MotorMapping::moveAbsoluteRc(master_, id, counts, 0, MOVE_ACCEL, MOVE_SPEED);
    noteResult(id, rc);                               // 在线/离线判定
    if (rc == MB_RC_OK) {
        last_rc_[id - 1] = 0;
    } else if (last_rc_[id - 1] != rc) {          // 只在失败原因变化时打印，避免刷屏
        last_rc_[id - 1] = rc;
        printf_color(PrintColor::Red,
                     "[DriveTask] 电机 %d 下发失败：%s（目标 %.3f° → %u 计数）\n",
                     id, modbusRcName(rc), motor_deg, counts);
    }
}

void ArmDriveTask::cleanup()
{
    printf_color(PrintColor::Green, "[DriveTask] cleanup\n");
}
