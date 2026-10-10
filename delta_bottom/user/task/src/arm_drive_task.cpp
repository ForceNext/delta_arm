#include "arm_drive_task.hpp"
#include <cmath>

// 运动参数（来自上位机实测）：加减速 200，速度 1000 RPM
constexpr uint8_t  MOVE_ACCEL = 200;
constexpr uint16_t MOVE_SPEED = 1000;
constexpr uint32_t OFFLINE_FAIL_LIMIT = 5;   // 连续失败 5 次判定掉线
constexpr uint32_t REPROBE_PERIOD    = 200;  // 停机后每 200 拍（≈1s）重探一次

ArmDriveTask::ArmDriveTask(PeriodicTaskManager* taskManager, float period, std::string name,
                           int argc, char** argv)
    : PeriodicTask(taskManager, period, name),
      port_(argc, argv),
      master_(port_),
      hw_cfg_((argc > 1) ? argv[1] : "configs/hardware_config.yaml"),
      kin_(Kinematics::fromYaml(hw_cfg_))
{
    master_.resp_timeout_ms = 20;

    int latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1) {
       printf("error: open /dev/cpu_dma_latency\n");
    }
    else
    {
        int latency_target_value = 0;
        int err = write(latency_target_fd, &latency_target_value, 4);
        if(err == -1)
        {
            printf("error: write /dev/cpu_dma_latency\n");
        }
        else
        {
            printf("set /dev/cpu_dma_latency %d\n",latency_target_value);
        }
    }
}

void ArmDriveTask::print_loop_rate()
{
    loop_counter_++;

    if (loop_counter_ == 1) 
    {
        time_start_sec_ = time_ctrl.getSeconds();
        return;
    }
    if (loop_counter_ >= 1000) 
    {
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
    loop_counter_ = 0;
    time_start_sec_ = 0.0;
    dt = 0.0;
    cycle_ = 0;
    time_ctrl.start();

    // 复位掉线停机状态
    for (int i = 0; i < 3; ++i) consec_fail_[i] = 0;
    emergency_stop_ = false;
    reprobe_counter_ = 0;

    shm_.connect();  // 连接共享内存：首次创建，其余附加

    // 探测并使能所有在线电机
    MotorMapping::loadMotorAddrs(hw_cfg_);
    MotorMapping::detectMotors(master_);
    MotorMapping::enableMotors(master_);
}

void ArmDriveTask::run()
{
    print_loop_rate();

    ArmCommand cmd;
    shm_.readCommand(cmd);

    double fk_xyz[3] = {cmd.x, cmd.y, cmd.z};   // 回显坐标，默认用目标值

    // ===== 掉线停机状态：不下发，只周期重探离线电机，等人工重启 =====
    if (emergency_stop_) {
        if (++reprobe_counter_ >= REPROBE_PERIOD) {
            reprobe_counter_ = 0;
            for (int id = 1; id <= 3; ++id) {
                if (MotorMapping::isOnline(id)) continue;
                if (MotorMapping::probeMotor(master_, id))
                    printf_color(PrintColor::Yellow,
                        "[DriveTask] 电机 %d 已恢复通信，请重启程序恢复运行\n", id);
            }
        }
        publishStatus(cmd, false, nullptr, fk_xyz);
        ++cycle_;
        return;
    }

    // ===== 正常控制 =====
    // 逆解：末端坐标 → 关节角 → ×传动比 → 计数；再正解回显
    uint32_t counts[3] = {0, 0, 0};
    bool reachable = kin_.kinematics_ik(cmd.x, cmd.y, cmd.z);
    if (reachable) {
        for (int i = 0; i < 3; ++i)
            counts[i] = (uint32_t)std::llround(
                MotorMapping::angleToMotorPos(kin_.motorTheta(i)));
        kin_.kinematics_fk(fk_xyz);
    }

    auto& g_err = MotorMapping::errs();

    // 逐台下发（按电机号 1/2/3），跳过离线电机，并统计连续失败
    if (cmd.mode == 1 && reachable) {
        for (int id = 1; id <= 3; ++id) {
            if (!MotorMapping::isOnline(id)) continue;

            modbus_timer_.start();
            bool ok = MotorMapping::moveAbsolute(master_, id, counts[id - 1],
                                                 0, MOVE_ACCEL, MOVE_SPEED);
            double t_ms = modbus_timer_.getMs();
            mb_sum_ms_ += t_ms;
            if (t_ms > mb_max_ms_) mb_max_ms_ = t_ms;
            ++mb_count_;

            if (!ok) {
                ++g_err[id - 1];
                if (++consec_fail_[id - 1] >= OFFLINE_FAIL_LIMIT) {
                    MotorMapping::setOnline(id, false);
                    printf_color(PrintColor::Red,
                        "[DriveTask] 电机 %d 连续失败 %u 次，判定掉线，停机！\n",
                        id, OFFLINE_FAIL_LIMIT);
                    emergency_stop_ = true;
                    break;
                }
            } else {
                consec_fail_[id - 1] = 0;
            }
        }
    }

    // 若刚触发停机：对仍在线电机刹车保持（不失能，避免末端掉落）
    if (emergency_stop_) {
        for (int id = 1; id <= 3; ++id)
            if (MotorMapping::isOnline(id))
                MotorMapping::stopMotor(master_, id);
        printf_color(PrintColor::Red,
            "[DriveTask] 已对在线电机执行刹车保持（不失能）\n");
    }

    publishStatus(cmd, reachable, counts, fk_xyz);

    ++cycle_;

    // 周期性打印 Modbus 事务耗时统计（每 1000 拍 ≈ 5 s）
    if (++mb_loops_ >= 1000) {
        if (mb_count_ > 0) {
            printf_color(PrintColor::Yellow,
                "[Modbus] 1000 loops: %u 次事务 | 平均 %.3f ms/次 | 最大 %.3f ms/次 | 每拍合计 %.3f ms\n",
                mb_count_, mb_sum_ms_ / mb_count_, mb_max_ms_, mb_sum_ms_ / 1000.0);
        } else {
            printf_color(PrintColor::Yellow,
                "[Modbus] 1000 loops: 0 次事务（无电机或 mode!=1）\n");
        }
        mb_sum_ms_ = 0.0;
        mb_max_ms_ = 0.0;
        mb_count_ = 0;
        mb_loops_ = 0;
    }
}

void ArmDriveTask::publishStatus(const ArmCommand& cmd, bool reachable,
                                 const uint32_t* counts, const double* fk_xyz)
{
    ArmStatus st{};
    st.seq = cycle_;
    st.state = (cmd.mode == 1 && reachable && !emergency_stop_) ? 1 : 0;
    for (int i = 0; i < 3; ++i) {
        st.online[i] = MotorMapping::isOnline(i + 1) ? 1 : 0;
        if (reachable && counts) {
            st.theta[i] = kin_.motorTheta(i);
            st.motor_pos[i] = (double)counts[i];
        }
    }
    st.x = fk_xyz[0]; st.y = fk_xyz[1]; st.z = fk_xyz[2];
    shm_.writeStatus(st);
}

void ArmDriveTask::cleanup()
{
    printf_color(PrintColor::Green, "[DriveTask] cleanup\n");
}