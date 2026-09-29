#include "arm_drive_task.hpp"

ArmDriveTask::ArmDriveTask(PeriodicTaskManager* taskManager, float period, std::string name)
    : PeriodicTask(taskManager, period, name)
{
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
    time_ctrl.start();

}

void ArmDriveTask::run()
{
    print_loop_rate();
    
}

void ArmDriveTask::cleanup()
{
    printf_color(PrintColor::Green, "[DriveTask] cleanup\n");
}