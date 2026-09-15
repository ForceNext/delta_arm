#ifndef _GNU_SOURCE
#define _GNU_SOURCE   // 启用 pthread_setaffinity_np 等 GNU 扩展
#endif
#include "serial_task.hpp"
#include <pthread.h>
#include <sched.h>
#include <cstring>
#include <iomanip>

// CRC-16/MODBUS（多项式 0x8005，按 Modbus 约定反转 0xA001，初始值 0xFFFF）
// 校验范围：从地址字节到数据区结束（不含帧尾 CRC 本身）
static uint16_t crc16Modbus(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

SerialTask::SerialTask(SerialPort& port) : port_(port)
{
    std::string path = "configs/thread_config.yaml";
    try
    {
        YAML::Node root = YAML::LoadFile(path);
        auto thread = root["interface_task"]["task_thread"];
        cpu_      = thread["bind_cpu"].as<int>();
        priority_ = thread["thread_priority"].as<int>();
        yaml_err = 0; // 成功读取

        std::cout << "读取线程配置成功: cpu=" << cpu_
                  << ", priority=" << priority_
                  << std::endl;
    }
    catch (const YAML::BadFile& e)
    {
        std::cerr << "yaml文件打开错误[" << path << "]: " << e.what() << "\n";
        yaml_err = 1; // 读取失败
    }
    catch (const YAML::ParserException& e)
    {
        std::cerr << "YAML 语法错误: " << e.what() << "\n";
        yaml_err = 2; // 语法错误
    }
    catch (const YAML::BadConversion& e)
    {
        std::cerr << "类型转换错误: " << e.what() << "\n";
        yaml_err = 3; // 类型转换错误
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "YAML 错误: " << e.what() << "\n";
        yaml_err = 4; // YAML 错误
    }
    catch (const std::exception& e)
    {
        std::cerr << "其他错误: " << e.what() << "\n";
        yaml_err = 5; // 其他错误
    }

    if (yaml_err != 0)
    {
        std::cerr << "线程配置读取失败,使用默认参数" << std::endl;
        cpu_      = 0;
        priority_ = 0;
    }
    start();
}

SerialTask::~SerialTask()
{
    stop();
}

void SerialTask::start()
{
    if (running_.load()) return;
    running_ = true;
    recv_thread_ = std::thread(&SerialTask::recvThread_, this);
}

void SerialTask::stop()
{
    if (!running_.load()) return;
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
}

void SerialTask::recvThread_()
{
    // 绑定 CPU 核（bind_cpu）
    if (cpu_ >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
            std::cerr << "[SerialTask] 设置 CPU 亲和性失败: " << strerror(errno) << "\n";
    }

    // 设置实时优先级（SCHED_FIFO，需要 root 权限或 CAP_SYS_NICE）
    if (priority_ > 0) {
        struct sched_param sp{};
        sp.sched_priority = priority_;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
            std::cerr << "[SerialTask] 设置 SCHED_FIFO 优先级失败: " << strerror(errno) << "\n";
    }

    // 事件驱动接收循环：select 阻塞等待数据，有数据立刻返回
    // 100ms 超时仅用于周期性检查 running_ 标志，以便优雅退出
    uint8_t buf[64];
    while (running_.load())
    {
        uint64_t ts = 0;
        ssize_t n = port_.read(buf, sizeof(buf), 100, &ts);
        if (n <= 0) continue;

        parseFrame_(buf, (size_t)n, ts);
    }
}

//原始数据解析（含 CRC 校验，校验失败直接丢弃，不更新 recv_obj_）
void SerialTask::parseFrame_(const uint8_t* buf, size_t len, uint64_t ts)
{
    if (len < 2) return;

    uint8_t addr    = buf[0];
    uint8_t funcode = buf[1];

    // 先确定本功能码下 CRC 覆盖长度与 CRC 在帧中的位置
    size_t crc_len = 0;   // CRC 覆盖字节数（addr .. data 区，不含 CRC 本身）
    size_t crc_pos = 0;   // CRC 低字节在 buf 中的下标

    if (funcode == 0x04) {
        // 读应答：addr + 04 + 字节数N + data[N] + crc(2)
        if (len < 3) return;
        uint8_t n = buf[2];
        crc_len   = 3 + n;
        crc_pos   = 3 + n;
    }
    else if (funcode == 0x06 || funcode == 0x10) {
        // 写应答：addr + funcode + 回显4字节 + crc(2)
        crc_len = 6;
        crc_pos = 6;
    }
    else {
        // 未知功能码：无法确定帧结构，直接丢弃
        std::cerr << "[SerialTask] 丢弃未知功能码帧: func=0x" << std::hex << std::setw(2)
                  << std::setfill('0') << (int)funcode << std::dec << " len=" << len << "\n";
        return;
    }

    // 帧长不足，取不出完整 CRC，丢弃
    if (len < crc_pos + 2) return;

    // 取出线上 CRC（低字节在前，组合为 uint16）并与计算结果比对
    uint16_t crc_rx  = buf[crc_pos] | ((uint16_t)buf[crc_pos + 1] << 8);
    uint16_t crc_cal = crc16Modbus(buf, crc_len);
    if (crc_rx != crc_cal) {
        std::cerr << "[SerialTask] CRC 校验失败，丢弃: 收到 0x" << std::hex << crc_rx
                  << "，计算 0x" << crc_cal << std::dec << "\n";
        return;
    }

    // 校验通过，写入 recv_obj_
    recv_obj_.addr    = addr;
    recv_obj_.funcode = funcode;
    recv_obj_.recv_ns = ts;
    recv_obj_.crc     = crc_rx;

    if (funcode == 0x04) {
        uint8_t n = buf[2];
        recv_obj_.datalen = (n > sizeof(recv_obj_.data)) ? sizeof(recv_obj_.data) : n;
        memcpy(recv_obj_.data, buf + 3, recv_obj_.datalen);
    }
    else { // 0x06 / 0x10
        recv_obj_.datalen = 4;
        memcpy(recv_obj_.data, buf + 2, 4);
    }

    // 调试打印（按需删除）
    std::cout << "[RX] addr=" << (int)recv_obj_.addr
              << " func=0x" << std::hex << std::setw(2) << std::setfill('0') << (int)recv_obj_.funcode
              << std::dec << " datalen=" << (int)recv_obj_.datalen
              << " ts=" << recv_obj_.recv_ns << "ns";
    for (uint8_t i = 0; i < recv_obj_.datalen; ++i)
        std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << (int)recv_obj_.data[i];
    std::cout << std::dec << "\n";
}
