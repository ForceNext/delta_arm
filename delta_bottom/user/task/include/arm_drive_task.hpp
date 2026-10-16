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

    int next_motor_ = 1;                   // （保留：等待上电阶段轮转探测用）
    int wait_probe_addr_ = 1;              // 等待上电阶段轮转探测的地址（1~3）
    int last_rc_[3] = {0, 0, 0};           // 每台上次失败原因（仅原因变化时打印，避免刷屏）
    uint64_t nontimeout_err_[3] = {0, 0, 0}; // 每台非超时失败累计（-2 地址/功能码、-3 CRC、>0 异常码）
    uint32_t mb_errcnt_ = 0;               // 统计窗口内非超时事务失败次数

    // —— 6 拍一轮：发M1→读M1→发M2→读M2→发M3→读M3（每拍只做 1 次 Modbus 事务）——
    // 一次事务约 2.9~3.2 ms，4 ms 周期内装不下"下发+回读"两次，
    // 因此把下发与回读拆到不同拍；一轮 6 拍 = 24 ms，每台下发/回读各约 41.7 Hz。
    int cycle_step_ = 0;                   // 0..5：偶数拍下发(id=step/2+1)，奇数拍回读同一台
    int32_t actual_pos_[3] = {0, 0, 0};    // 实测电机位置（计数，下标 = 电机号-1）
    double  actual_joint_[3] = {0, 0, 0};  // 实测关节角（度，= 计数 ÷(51200/360) ÷ 减速比）
    bool    actual_valid_[3] = {false, false, false}; // 该台是否已成功回读过
    bool    actual_pose_valid_ = false;    // 三台都有实测值，x/y/z 为实测正解结果
    double  actual_xyz_[3] = {0, 0, 0};    // 实测正解出的末端坐标（mm）
    uint32_t mb_disp_ = 0, mb_read_ = 0;   // 统计窗口内下发/回读事务次数

    // 一次事务失败的统一处理：仅"超时（无应答）"计入掉线判定，其余只统计+打印原因
    void handleTxFailure(int id, int rc, const char* what);

    // 发布状态到共享内存（x/y/z 在有三台实测值时改为实测正解结果）
    void publishStatus(const ArmCommand& cmd, bool reachable,
                       const uint32_t* counts, const double* fk_xyz);
};

#endif