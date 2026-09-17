#pragma once

#include <cstdint>
#include <vector>

#include "serial_port.hpp"

// Modbus RTU 主站：同步事务封装。
// 每个调用 = 组帧 + CRC + 发送 + 两段超时等应答 + 校验，返回是否成功。
class ModbusMaster
{
public:
    explicit ModbusMaster(SerialPort& port);

    // 读保持寄存器 0x03 / 输入寄存器 0x04：count 个寄存器，结果按大端写入 out[]
    bool readHoldingRegisters(uint8_t addr, uint16_t start, uint16_t count, uint16_t* out);
    bool readInputRegisters(uint8_t addr, uint16_t start, uint16_t count, uint16_t* out);

    // 写单寄存器 0x06 / 写多寄存器 0x10
    bool writeRegister(uint8_t addr, uint16_t reg, uint16_t val);
    bool writeRegisters(uint8_t addr, uint16_t start, uint16_t count, const uint16_t* vals);

    // 32 位便捷封装：写 2 个寄存器（高 16 位在前）
    bool writeRegister32(uint8_t addr, uint16_t reg, uint32_t val);

    // 超时参数（默认值，可按实测调整）
    int resp_timeout_ms = 50;   // 等首字节
    int byte_timeout_ms = 5;    // 帧内字节间隔

private:
    // 一次事务：发 req，等应答并校验 addr/func/CRC，成功后把「func+数据区」写入 rsp。
    // 返回 0=成功, -1=超时/长度不足, -2=地址或功能码不匹配, -3=CRC 错, >0=从站异常码
    int transact_(const std::vector<uint8_t>& req, uint8_t addr, uint8_t func,
                  std::vector<uint8_t>* rsp);
    bool readRegisters_(uint8_t addr, uint8_t func, uint16_t start, uint16_t count, uint16_t* out);

    SerialPort& port_;
};
