#ifndef ARM_DRIVE_TASK_HPP
#define ARM_DRIVE_TASK_HPP

#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"
#include "MotorMapping.hpp"
#include "Kinematics.hpp"
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

    Timer modbus_timer_;              // 测量单次 Modbus 事务耗时
    double mb_max_ms_ = 0.0;          // 统计窗口内单次事务最大耗时（ms）
    double mb_sum_ms_ = 0.0;          // 统计窗口内事务总耗时（ms）
    uint32_t mb_count_ = 0;           // 统计窗口内事务次数
    uint32_t mb_loops_ = 0;           // 统计窗口内循环次数

    SerialPort port_;                      // 串口（按 argc/argv 读取 hardware_config.yaml）
    ModbusMaster master_;                  // Modbus 主站（基于 port_）
    ArmSharedMemory shm_;                  // 上下位机共享内存（命令/状态）
    uint64_t cycle_ = 0;                   // 控制循环计数（状态 seq）
    std::string hw_cfg_;                   // 硬件配置文件路径（串口 + 电机地址 + 关节参数）
    Kinematics kin_;                       // delta 正逆解（逆解：末端 xyz → 关节角）

    uint32_t consec_fail_[3] = {0, 0, 0};  // 每台连续失败次数（下标 = 电机号-1）
    bool emergency_stop_ = false;          // 掉线停机标志
    uint32_t reprobe_counter_ = 0;         // 周期探测/重探计数
    bool motors_ready_ = false;            // 所有电机已在线并使能
    uint8_t prev_online_mask_ = 0;         // 上次打印的在线位掩码（避免刷屏）

    // 发布状态到共享内存（online 反映真实在线状态）
    void publishStatus(const ArmCommand& cmd, bool reachable,
                       const uint32_t* counts, const double* fk_xyz);
};

#endif