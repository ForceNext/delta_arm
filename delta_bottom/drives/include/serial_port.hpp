#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <sys/types.h>
#include "yaml-cpp/yaml.h"
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>   // errno
#include <termios.h>

class SerialPort
{
private:
   struct DriveParam
   {
       std::string port_name;  // 串口名称
       int baud_rate;          // 波特率
   };
   DriveParam param;
   char yaml_err = 0;
   int fd_ = -1;
   bool serial_opened_ = false;
public:
    SerialPort(int argc, char** argv);
    ~SerialPort();
    bool setInterfaceAttrs(int baudrate);
    int baudrateToSpeed(int baudrate);
    bool open(const std::string& port, int baudrate);
    void close();
    ssize_t write(const uint8_t* data, size_t len);
    ssize_t write(const std::vector<uint8_t>& data);
    ssize_t read(uint8_t* buffer, size_t len, int timeout_ms = 500,
                 uint64_t* first_byte_ns = nullptr,
                 int byte_timeout_ms = 20);  // 首字节后的帧内字节间隔超时（默认 20ms）
    // 精确读取 len 字节：首字节用 timeout_ms 等待，后续字节用 byte_timeout_ms 等待，
    // 读满 len 字节立即返回，不做帧尾静默等待（供 ModbusMaster 按期望长度收帧用）。
    ssize_t readExact(uint8_t* buffer, size_t len, int timeout_ms, int byte_timeout_ms = 20);

    // 清空接收缓冲（每次发帧前调用，防止残留字节导致帧错位）
    void flushInput();

    // 等总线静默：丢弃收到的字节，直到连续 quiet_ms 内没有新数据为止。
    // 用于 RS485 帧间间隔——上一帧的尾巴、迟到或多余的应答若不清掉，
    // 会被下一次事务当成"自己的应答"，造成地址/CRC 不匹配并连锁污染后续帧。
    // 返回 true = 已静默；false = 在 max_ms 内始终有数据（超时放弃）。
    bool waitIdle(int quiet_ms = 1, int max_ms = 20);
    bool Get_SerialOpened(){return serial_opened_;}
};