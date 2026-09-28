#ifndef MOTOR_MAPPING_HPP
#define MOTOR_MAPPING_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <iostream>

#include "yaml-cpp/yaml.h"
#include "modbus_master.hpp"

#define MOTOR_RESOLUTION 51200.0  //电机单圈分辨率

// ---------------------------------------------------------------------------
// 正点原子 PDxxS1 命令码：驱动把「寄存器起始地址」当作命令码用（高字节补 0），
// 读写都走标准 Modbus 帧（读=0x03/0x04，写=0x06/0x10）。
// ---------------------------------------------------------------------------
namespace Cmd {
    constexpr uint16_t READ_POS   = 0x002A;  // 读实时位置（int32，51200=一圈），也用于在线探测
    constexpr uint16_t READ_SPEED = 0x0029;  // 读实时转速（int16，单位 RPM）
    constexpr uint16_t ABS_POS    = 0x00F2;  // 闭环绝对位置模式：方向(1)+加减速(1)+速度(2)+位置(4)
    constexpr uint16_t ENABLE     = 0x00FA;  // 使能：0=使能 / 1=失能
    constexpr uint16_t SET_MODE   = 0x0062;  // 设置工作模式：0=通信位置 … 7=回零
    constexpr uint16_t REL_POS      = 0x00F3;  // 相对位置模式控制
    constexpr uint16_t CLEAR_STATUS = 0x00FB;  // 清除状态（堵转/刹车/失能）
    constexpr uint16_t STOP         = 0x00FC;  // 立即停止（刹车）
}

// ---------------------------------------------------------------------------
// 电机映射 / 控制封装（正点原子 PDxxS1 闭环步进驱动器）：
//   - 扫描 / 探测在线电机
//   - 使能、工作模式、绝对/相对位置、刹车、清除状态等运动控制
//   - 关节角度（度）↔ 电机计数（51200 = 一圈）的换算
// 均为静态方法，通过传入的 ModbusMaster& 操作串口；在线地址与错误计数保存在静态成员里。
// ---------------------------------------------------------------------------
class MotorMapping
{
public:
    MotorMapping() = default;
    ~MotorMapping() = default;

    // —— 扫描 / 探测电机 ——
    static void loadMotorAddrs(const std::string& path);  // 从 yaml 读候选地址到 g_cfg_addr
    static void detectMotors(ModbusMaster& master);       // 逐个读实时位置，探测在线电机

    // 探测结果访问（供控制循环使用）
    static const std::vector<uint8_t>& addrs() { return g_addr; }  // 在线从站地址
    static std::vector<uint64_t>& errs() { return g_err; }         // 每台累计失败次数

    static double motorPosToAngle(double pos);   // 电机计数 → 关节角度（度）
    static double angleToMotorPos(double angle); // 关节角度（度）→ 电机计数

    static bool SetMotorMode(ModbusMaster& master, int motorId, int mode);

    // —— 电机使能 ——
    static bool setMotorEnable(ModbusMaster& master, int motorId, bool enable);  // 单台使能/失能
    static void enableMotors(ModbusMaster& master);                              // 使能所有在线电机

    // —— 运动控制 ——
    static bool moveAbsolute(ModbusMaster& master, int motorId, uint32_t position,
                             uint8_t direction = 0, uint8_t accel = 200, uint16_t speed = 1000);
    static bool moveRelative(ModbusMaster& master, int motorId, uint32_t offset,
                             uint8_t direction = 0, uint8_t accel = 200, uint16_t speed = 1000);
    static bool stopMotor(ModbusMaster& master, int motorId);    // 立即停止（刹车）
    static bool clearStatus(ModbusMaster& master, int motorId);  // 清除状态（堵转/刹车/失能）

private:
    static constexpr double DEG2CNT = MOTOR_RESOLUTION / 360.0;
    static inline std::vector<uint8_t> g_cfg_addr = {1, 2, 3};  // 配置里的所有从站地址
    static inline std::vector<uint8_t> g_addr;                  // 实际在线、要控制的从站地址
    static inline std::vector<uint64_t> g_err;                  // 每台累计失败次数
};

#endif // MOTOR_MAPPING_HPP
