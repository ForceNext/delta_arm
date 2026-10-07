#include "arm_drive_task.hpp"
#include <cmath>

// 运动参数（来自上位机实测）：加减速 200，速度 1000 RPM
constexpr uint8_t  MOVE_ACCEL = 200;
constexpr uint16_t MOVE_SPEED = 1000;

ArmDriveTask::ArmDriveTask(PeriodicTaskManager* taskManager, float period, std::string name,
                           int argc, char** argv)
    : PeriodicTask(taskManager, period, name),
      port_(argc, argv),
      master_(port_),
      hw_cfg_((argc > 1) ? argv[1] : "configs/hardware_config.yaml")
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

    shm_.connect();  // 连接共享内存：首次创建，其余附加

    // 探测并使能所有在线电机
    MotorMapping::loadMotorAddrs(hw_cfg_);
    MotorMapping::detectMotors(master_);
    MotorMapping::enableMotors(master_);
}

void ArmDriveTask::run()
{
    print_loop_rate();

    // 1) 读上位机命令：xyz 直接作为三个电机的目标角度（单位：度）
    ArmCommand cmd;
    shm_.readCommand(cmd);

    // 共享内存 → 三个目标角度
    target_[0] = cmd.x;
    target_[1] = cmd.y;
    target_[2] = cmd.z;

    // 2) 角度 → 电机计数，逐台下发
    uint32_t counts[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i)
        counts[i] = (uint32_t)std::llround(MotorMapping::angleToMotorPos(target_[i]));

    const auto& g_addr = MotorMapping::addrs();
    auto& g_err = MotorMapping::errs();

    if (cmd.mode == 1) {
        for (size_t i = 0; i < g_addr.size() && i < 3; ++i) {
            // 下发绝对位置：方向 0=正转，加减速 MOVE_ACCEL，速度 MOVE_SPEED
            if (!MotorMapping::moveAbsolute(master_, g_addr[i], counts[i],
                                            0, MOVE_ACCEL, MOVE_SPEED))
                ++g_err[i];
        }
    }

    // 3) 发布状态（在线 + 目标位置 + 回显坐标）
    {
        ArmStatus st{};
        st.seq = cycle_;
        st.state = (cmd.mode == 1) ? 1 : 0;
        for (size_t i = 0; i < g_addr.size() && i < 3; ++i) {
            st.online[i] = 1;
            st.theta[i] = target_[i];
            st.motor_pos[i] = (double)counts[i];
        }
        st.x = cmd.x; st.y = cmd.y; st.z = cmd.z;
        shm_.writeStatus(st);
    }

    ++cycle_;
}

void ArmDriveTask::cleanup()
{
    printf_color(PrintColor::Green, "[DriveTask] cleanup\n");
}