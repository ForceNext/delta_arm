#pragma once

#include <cstdint>
#include <vector>

#include "serial_port.hpp"

// transact_ 的返回码：对外暴露，便于上层区分「超时 / 帧错 / 从站异常」，而不是只拿到一个 bool
enum ModbusRc : int {
    MB_RC_OK      =  0,   // 成功
    MB_RC_TIMEOUT = -1,   // 超时，或应答长度不足
    MB_RC_BADADDR = -2,   // 从站地址或功能码不匹配
    MB_RC_BADCRC  = -3,   // CRC 校验错
    // > 0：从站异常码（异常应答的第 3 字节，如 0x01 非法功能、0x04 从站设备故障）
};

// 返回码的可读描述（含常见 Modbus 异常码含义）
const char* modbusRcName(int rc);

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

    // —— 带返回码的版本（返回值见 ModbusRc，供上层记录失败原因）——
    int readInputRegistersRc(uint8_t addr, uint16_t start, uint16_t count, uint16_t* out);
    int writeRegisterRc(uint8_t addr, uint16_t reg, uint16_t val);
    int writeRegistersRc(uint8_t addr, uint16_t start, uint16_t count, const uint16_t* vals);

    // 超时参数（默认值，可按实测调整）
    int resp_timeout_ms = 50;   // 等首字节
    int byte_timeout_ms = 5;    // 帧内字节间隔
    int idle_gap_ms = 1;        // 发帧前要求的总线静默时间（帧间间隔；异常收尾按 4 倍等待）

private:
    // 一次事务：发 req，按期望长度收应答并校验 addr/func/CRC，成功后把「func+数据区」写入 rsp。
    // exp_len：期望的完整应答帧长度（含 addr 与 CRC），用于按长度精确收帧，避免帧尾静默等待。
    // 返回 0=成功, -1=超时/长度不足, -2=地址或功能码不匹配, -3=CRC 错, >0=从站异常码
    int transact_(const std::vector<uint8_t>& req, uint8_t addr, uint8_t func,
                  uint16_t exp_len, std::vector<uint8_t>* rsp);
    int readRegisters_(uint8_t addr, uint8_t func, uint16_t start, uint16_t count, uint16_t* out);

    SerialPort& port_;
    // 上一次事务是否异常收尾（可能留下未读完的字节）——只有这种情况才在发帧前等总线静默，
    // 正常路径不额外等待，避免把 200Hz 周期预算吃掉。
    bool need_drain_ = false;
};
