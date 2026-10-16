#ifndef ARM_DRIVE_TASK_HPP
#define ARM_DRIVE_TASK_HPP

#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"
#include "MotorMapping.hpp"
#include "serial_port.hpp"
#include "modbus_master.hpp"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"

// 机械臂驱动任务（骨架）
// 说明：控制循环 run() 已清空，只保留周期统计 print_loop_rate()；运动逻辑待重写。
// 保留的成员是"底层通道"：串口 + Modbus 主站 + 共享内存 + 硬件配置路径。
class ArmDriveTask : public PeriodicTask
{
public:
    ArmDriveTask(PeriodicTaskManager* taskManager, float period, std::string name,
                 int argc, char** argv);
    ~ArmDriveTask() override = default;
    void init() override;
    void run() override;
    void cleanup() override;
    void print_loop_rate();
private:
    Timer time_ctrl;                       // 周期统计用（print_loop_rate）
    uint16_t loop_counter_ = 0;
    double time_start_sec_ = 0.0;

    SerialPort port_;                      // 串口（按 argc/argv 读取 hardware_config.yaml）
    ModbusMaster master_;                  // Modbus 主站（基于 port_）
    ArmSharedMemory shm_;                  // 上下位机共享内存（命令/状态）
    std::string hw_cfg_;                   // 硬件配置文件路径（串口 + 电机地址 + 减速比）
};

#endif
