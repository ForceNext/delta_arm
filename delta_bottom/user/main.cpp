#include <iostream>
#include <vector>
#include <cmath>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <cerrno>
#include <string>
#include <thread>

#include "yaml-cpp/yaml.h"
#include "modbus_master.hpp"
#include "MotorMapping.hpp"
#include "Utilities/SharedMemory.h"
#include "Utilities/PeriodicTask.h"
#include "arm_drive_task.hpp"

ArmDriveTask *arm_drive_task = nullptr;

// 运行标志：信号处理函数置 0 实现优雅退出（sig_atomic_t 保证信号安全）
static volatile sig_atomic_t g_running = 1;

static void onSignal(int) { g_running = 0; }

int main(int argc, char** argv)
{
    printf_color(PrintColor::Green,"Hello, Delta Arm!\n");
    PeriodicTaskManager *taskManager = PeriodicTaskManager::get_instance();
    // 读取线程配置（默认值：周期 5ms、绑定 CPU4、优先级 75）
    float thread_t = 0.005f;
    int cpu_id = 4;
    int thread_priority = 75;

    std::string path = (argc > 1) ? argv[1] : "configs/thread_config.yaml";
    try
    {
        YAML::Node root = YAML::LoadFile(path);
        auto interface_task = root["interface_task"];
        auto task_thread = interface_task["task_thread"];
        thread_t = task_thread["thread_t"].as<float>();
        cpu_id = task_thread["bind_cpu"].as<int>();
        thread_priority = task_thread["thread_priority"].as<int>();
    }
    catch (const std::exception& e)
    {
        std::cerr << "读取线程配置失败(" << e.what() << ")，使用默认值\n";
    }

    std::cout << "thread_t=" << thread_t
              << " cpu_id=" << cpu_id
              << " thread_priority=" << thread_priority << std::endl;

    arm_drive_task = new ArmDriveTask(taskManager, thread_t, "ArmDriveTask", argc, argv);
    arm_drive_task->set_bind_cpu(cpu_id);
    arm_drive_task->set_sched_priority(thread_priority);

    // 安装信号处理：SIGINT/SIGTERM 触发优雅退出
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    arm_drive_task->start();

    std::cout << "运行中... 按 Ctrl+C 退出\n";
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "收到退出信号，停止所有任务...\n";
    taskManager->stopAll();   // 内部 join 各线程并执行 cleanup
    return 0;
}