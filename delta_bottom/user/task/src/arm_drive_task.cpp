#include "arm_drive_task.hpp"

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

    // 上下位机共享内存（每次启动清零）
    shm_.connect();

    // 串口 + 电机地址 + 减速比（运动逻辑待重写）
    MotorMapping::loadMotorAddrs(hw_cfg_);
    MotorMapping::loadGearRatios(hw_cfg_);
}

void ArmDriveTask::run()
{
    print_loop_rate();
}

void ArmDriveTask::cleanup()
{
    printf_color(PrintColor::Green, "[DriveTask] cleanup\n");
}
