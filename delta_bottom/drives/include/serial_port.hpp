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
public:
    SerialPort(int argc, char** argv);
    ~SerialPort();
    bool setInterfaceAttrs(int baudrate);
    int baudrateToSpeed(int baudrate);
    bool open(const std::string& port, int baudrate);
    void close();
    ssize_t write(const uint8_t* data, size_t len);
    ssize_t write(const std::vector<uint8_t>& data);
    ssize_t read(uint8_t* buffer, size_t len, int timeout_ms = 500);
    // 清空接收缓冲（每次发帧前调用，防止残留字节导致帧错位）
    void flushInput();
};