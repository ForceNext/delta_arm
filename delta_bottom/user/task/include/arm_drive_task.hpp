#ifndef ARM_DRIVE_TASK_HPP
#define ARM_DRIVE_TASK_HPP

#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"
#include "MotorMapping.hpp"
#include "serial_port.hpp"
#include "modbus_master.hpp"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"

// 机械臂驱动任务
// 当前控制循环 run()：读共享内存里的三个目标角度，每个周期只给一台电机发绝对位置指令（1→2→3 轮转）。
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

    // —— 下发逻辑状态 ——
    uint64_t cycle_ = 0;                   // 循环计数（发布到 ArmStatus.seq，上位机靠它判活）
    bool drives_ready_ = false;            // 三台已探测到、已初始化并使能
    int next_motor_ = 1;                   // 本拍要下发的电机号（1→2→3 轮转）
    int next_probe_ = 1;                   // 等待上电时轮转探测的电机号
    uint32_t probe_tick_ = 0;              // 探测节拍计数（每 PROBE_PERIOD 拍探一台）
    int last_rc_[3] = {0, 0, 0};           // 每台上次失败原因（只在变化时打印，避免刷屏）
    uint32_t consec_timeout_[3] = {0, 0, 0};   // 每台连续"无应答"次数
    uint32_t offline_probe_tick_ = 0;      // 离线重探节拍计数（每 OFFLINE_REPROBE_PERIOD 拍探一台）

    // 探测 → 初始化 → 使能；全部完成返回 true
    bool ensureDrivesReady();

    // 记录一次事务结果（下发或探测用）：
    //   有应答（成功 / CRC 错 / 从站异常码）→ 认为还在线，连续无应答计数清零；
    //   连续无应答达到 TIMEOUT_BEFORE_PROBE 次 → 专门探一次，确认是否真的掉线。
    void noteResult(int id, int rc);

    // 每拍发布状态：seq 递增（上位机/工具靠它判断底层是否在运行）、state、online[]
    // 注：还没有实测回读，所以 state 的 bit1（实测有效）暂不置位，
    //     theta/motor_pos/joint_theta 保持 0；等加回读后再填实测值。
    void publishStatus(const ArmCommand& cmd);
};

#endif
