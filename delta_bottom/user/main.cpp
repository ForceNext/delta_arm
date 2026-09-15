#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <chrono>
#include <cstdint>
#include "serial_task.hpp"

// CRC-16/MODBUS（与 serial_task.cpp 内的实现一致；发送侧组帧用，之后可提到公共头复用）
static uint16_t crc16Modbus(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// 组读寄存器请求帧：addr + func + start(2) + count(2) + crc(2)
static std::vector<uint8_t> makeReadReq(uint8_t addr, uint8_t func, uint16_t start, uint16_t count)
{
    std::vector<uint8_t> f = {
        addr, func,
        (uint8_t)(start >> 8), (uint8_t)(start & 0xFF),
        (uint8_t)(count >> 8), (uint8_t)(count & 0xFF),
    };
    uint16_t crc = crc16Modbus(f.data(), f.size());
    f.push_back((uint8_t)(crc & 0xFF));   // CRC 低字节在前
    f.push_back((uint8_t)(crc >> 8));
    return f;
}

int main(int argc, char** argv)
{
    SerialPort port(argc, argv);   // 打开串口
    SerialTask task(port);         // 构造即启动接收线程

    const uint8_t  slave = 0x01;   // 从站地址
    const uint16_t start = 0x20;   // 版本号所在输入寄存器地址
    const uint16_t count = 1;

    // 1) 登记事务：告诉接收线程"接下来读输入寄存器 0x20，共 1 个"，
    //    应答帧不含起始地址，接收线程据此把数据定位到寄存器表。
    task.setPendingRequest(0x04, start, count);

    // 2) 组帧并发送
    auto cmd = makeReadReq(slave, 0x04, start, count);
    std::cout << "[TX] " << cmd.size() << " 字节:";
    for (auto b : cmd)
        std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << (int)b;
    std::cout << std::dec << "\n";

    port.flushInput();   // 发送前清接收缓冲，防止残留字节
    ssize_t n = port.write(cmd);
    if (n != (ssize_t)cmd.size()) {
        std::cerr << "发送失败：期望 " << cmd.size() << " 字节，实际 " << n << "\n";
        return -1;
    }

    // 3) 等待接收线程处理（验证用简单 sleep；生产环境应改为超时轮询/条件变量）
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 4) 从寄存器表读结果，验证接收线程是否正确解析、校验并落地
    uint16_t ver = task.inputReg(start);
    std::cout << "[VERIFY] inputReg(0x" << std::hex << start << std::dec
              << ") = " << ver << " (0x" << std::hex << ver << std::dec << ")\n";

    uint64_t ns = task.lastRecvNs();
    std::cout << "[VERIFY] lastRecvNs = " << ns
              << (ns ? "  -> 已收到至少一帧应答" : "  -> 未收到任何有效应答") << "\n";

    return 0;   // 析构 SerialTask 时自动 stop() 并回收接收线程
}
