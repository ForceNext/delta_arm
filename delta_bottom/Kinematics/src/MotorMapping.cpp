// MotorMapping 实现：正点原子 PDxxS1 闭环步进驱动器的电机映射与控制封装。
// 各命令码（寄存器起始地址）定义见 MotorMapping.hpp 的 Cmd 命名空间。
#include "MotorMapping.hpp"

#include <iostream>

#include "yaml-cpp/yaml.h"

// 设置工作模式（功能码 0x06，寄存器 0x62）：
//   0=通信位置  1=通信速度  2=通信力矩  3=脉冲
//   4=脉宽位置  5=脉宽速度  6=脉宽力矩  7=回零
bool MotorMapping::SetMotorMode(ModbusMaster& master, int motorId, int mode)
{
    if (motorId < 1 || motorId > 3) {
        return false; // 无效的电机ID
    }
    if (mode < 0 || mode > 7) {
        return false; // 无效的工作模式
    }
    // motorId 直接作为从站地址（默认 1/2/3）
    return master.writeRegister((uint8_t)motorId, Cmd::SET_MODE, (uint16_t)mode);
}

// 单台电机使能/失能（功能码 0x06，寄存器 0xFA）：0=使能，1=失能
bool MotorMapping::setMotorEnable(ModbusMaster& master, int motorId, bool enable)
{
    if (motorId < 1 || motorId > 3) {
        return false; // 无效的电机ID
    }
    return master.writeRegister((uint8_t)motorId, Cmd::ENABLE, enable ? 0x0000 : 0x0001);
}

// 使能所有在线电机（启动时调用）
void MotorMapping::enableMotors(ModbusMaster& master)
{
    for (uint8_t a : g_addr)
        master.writeRegister(a, Cmd::ENABLE, 0x0000);
}

// 发送绝对位置控制命令（功能码 0x10，寄存器 0xF2）：
//   方向(1) + 加减速(1) + 速度(2) + 位置(4)，共 4 个寄存器
//   direction：0=正转，1=反转；accel：0~200；speed：0~6000 RPM；position：计数，51200=一圈
bool MotorMapping::moveAbsolute(ModbusMaster& master, int motorId, uint32_t position,
                                uint8_t direction, uint8_t accel, uint16_t speed)
{
    if (motorId < 1 || motorId > 3) {
        return false; // 无效的电机ID
    }
    uint16_t regs[4];
    regs[0] = (uint16_t)((direction << 8) | accel);  // 方向 | 加减速
    regs[1] = speed;                                 // 速度 RPM
    regs[2] = (uint16_t)(position >> 16);            // 位置高 16 位
    regs[3] = (uint16_t)(position & 0xFFFF);         // 位置低 16 位
    return master.writeRegisters((uint8_t)motorId, Cmd::ABS_POS, 4, regs);
}

// 发送相对位置控制命令（功能码 0x10，寄存器 0xF3）：结构与绝对位置相同，位置为相对偏移
bool MotorMapping::moveRelative(ModbusMaster& master, int motorId, uint32_t offset,
                                uint8_t direction, uint8_t accel, uint16_t speed)
{
    if (motorId < 1 || motorId > 3) {
        return false; // 无效的电机ID
    }
    uint16_t regs[4];
    regs[0] = (uint16_t)((direction << 8) | accel);  // 方向 | 加减速
    regs[1] = speed;                                 // 速度 RPM
    regs[2] = (uint16_t)(offset >> 16);              // 位置高 16 位
    regs[3] = (uint16_t)(offset & 0xFFFF);           // 位置低 16 位
    return master.writeRegisters((uint8_t)motorId, Cmd::REL_POS, 4, regs);
}

// 立即停止（刹车）（功能码 0x06，寄存器 0xFC，数据 0x0001）
// 注意：刹停后需及时调用 clearStatus 清除状态，否则电机可能严重发烫
bool MotorMapping::stopMotor(ModbusMaster& master, int motorId)
{
    if (motorId < 1 || motorId > 3) {
        return false; // 无效的电机ID
    }
    return master.writeRegister((uint8_t)motorId, Cmd::STOP, 0x0001);
}

// 清除状态（堵转/刹车/失能）（功能码 0x06，寄存器 0xFB，数据 0x0001）
bool MotorMapping::clearStatus(ModbusMaster& master, int motorId)
{
    if (motorId < 1 || motorId > 3) {
        return false; // 无效的电机ID
    }
    return master.writeRegister((uint8_t)motorId, Cmd::CLEAR_STATUS, 0x0001);
}

// 电机计数 → 关节角度（度）：51200 计数 = 一圈 360°
double MotorMapping::motorPosToAngle(double pos)
{
    return pos / DEG2CNT;
}

// 关节角度（度）→ 电机计数
double MotorMapping::angleToMotorPos(double angle)
{
    return angle * DEG2CNT;
}

// 从 yaml 读电机候选地址（覆盖默认 {1,2,3}）
void MotorMapping::loadMotorAddrs(const std::string& path)
{
    try {
        YAML::Node root = YAML::LoadFile(path);
        std::vector<uint8_t> a;
        for (const auto& m : root["motors"])
            a.push_back((uint8_t)m["addr"].as<int>());
        if (!a.empty()) g_cfg_addr = a;
    } catch (const std::exception& e) {
        std::cerr << "读取电机地址失败(" << e.what() << ")，使用默认 {1,2,3}\n";
    }
}

// 探测在线电机：逐个读实时位置（0x2A），能应答的才纳入控制列表
void MotorMapping::detectMotors(ModbusMaster& master)
{
    g_addr.clear();
    for (uint8_t a : g_cfg_addr) {
        uint16_t buf[2] = {0, 0};
        if (master.readInputRegisters(a, Cmd::READ_POS, 2, buf))
            g_addr.push_back(a);
    }
    if (g_addr.empty()) {
        std::cerr << "未探测到任何在线电机，退回配置地址\n";
        g_addr = g_cfg_addr;
    }
    g_err.assign(g_addr.size(), 0);
    std::cout << "探测到 " << g_addr.size() << " 台在线电机\n";
}


