#ifndef ARM_DRIVE_TASK_HPP
#define ARM_DRIVE_TASK_HPP

#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"
#include "MotorMapping.hpp"
#include "serial_port.hpp"
#include "modbus_master.hpp"
#include "yaml-cpp/yaml.h"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"

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
    Timer time_ctrl;
    uint16_t loop_counter_;
    double time_start_sec_;
    float dt;

    SerialPort port_;                      // 串口（按 argc/argv 读取 hardware_config.yaml）
    ModbusMaster master_;                  // Modbus 主站（基于 port_）
    ArmSharedMemory shm_;                  // 上下位机共享内存（命令/状态）
    double target_[3] = {0.0, 0.0, 0.0};   // 由共享内存 xyz 解析出的三个目标角度（度）
    uint64_t cycle_ = 0;                   // 控制循环计数（状态 seq）
    std::string hw_cfg_;                   // 硬件配置文件路径（串口 + 电机地址）
};

#endif