#ifndef SERIAL_HPP
#define SERIAL_HPP

#include "iostream"
#include "serial_port.hpp"
#include "Utilities/Timer.h"
#include "Utilities/PeriodicTask.h"

class SerialTask
{
    private:
        struct SERIAL_OBJ_
        {
            uint8_t  addr;        // 从机地址（1 字节）
            uint8_t  funcode;     // 功能码（1 字节）：04H 读输入寄存器 / 06H 写单个 / 10H 写多个
            uint8_t  datalen;     // 数据区字节数（04H 读应答的"字节数"字段）
            uint8_t  data[40];    // 数据区内容：读指令最大 40 字节（见 5.3.18 读取系统参数）
            uint16_t crc;         // CRC16 校验值（线路上低字节在前，组合为 (高字节<<8)|低字节）
            uint64_t recv_ns;     // 收到首个字节的单调时钟纳秒时间戳（CLOCK_MONOTONIC）
        };
    
    public:
        void recvThread_();
};

#endif // SERIAL_HPP