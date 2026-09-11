#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstdint>
#include "serial_port.hpp"

static void printHex(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        std::cout << std::hex << std::uppercase
                  << std::setw(2) << std::setfill('0') << (int)data[i] << ' ';
    }
    std::cout << std::dec;
}

int main(int argc, char** argv)
{
    SerialPort serialPort(argc, argv);

    // 读取软硬件版本：从站地址 01，功能码 04（读输入寄存器），起始寄存器 0x0020，数量 1
    // 帧：01 04 00 20 00 01 + CRC(30 00)
    std::vector<uint8_t> cmd = {0x01, 0x04, 0x00, 0x20, 0x00, 0x01, 0x30, 0x00};

    serialPort.flushInput();   // 发送前清空接收缓冲，防止残留字节

    std::cout << "[TX] " << cmd.size() << " 字节: ";
    printHex(cmd.data(), cmd.size());
    std::cout << "\n";

    ssize_t n = serialPort.write(cmd);
    if (n != (ssize_t)cmd.size()) {
        std::cerr << "发送失败：期望 " << cmd.size() << " 字节，实际 " << n << "\n";
        return -1;
    }

    uint8_t buf[256] = {0};
    ssize_t r = serialPort.read(buf, sizeof(buf), 500);  // 500ms 超时
    if (r <= 0) {
        std::cout << "[RX] 未收到数据 (ret=" << r << ")\n";
        return 0;
    }

    std::cout << "[RX] " << r << " 字节: ";
    printHex(buf, (size_t)r);
    std::cout << "\n";

    return 0;
}
