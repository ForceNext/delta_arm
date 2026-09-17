#pragma once

#include <cstdint>
#include <cstddef>

// CRC-16/MODBUS（多项式 0x8005，按 Modbus 约定反转 0xA001，初始值 0xFFFF）
// 校验范围：从地址字节到数据区结束（不含帧尾 CRC 本身）。
inline uint16_t crc16Modbus(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}
