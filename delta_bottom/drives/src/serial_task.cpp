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

void SerialTask::setPendingRequest(uint8_t func, uint16_t start_addr, uint16_t reg_count)
{
    std::lock_guard<std::mutex> lock(mtx_);
    pending_.active     = true;
    pending_.func       = func;
    pending_.start_addr = start_addr;
    pending_.reg_count  = reg_count;
}

uint16_t SerialTask::inputReg(uint16_t addr) const
{
    if (addr >= SlaveRegisters::SIZE) return 0;
    std::lock_guard<std::mutex> lock(mtx_);
    return regs_.input_regs[addr];
}

uint16_t SerialTask::holdingReg(uint16_t addr) const
{
    if (addr >= SlaveRegisters::SIZE) return 0;
    std::lock_guard<std::mutex> lock(mtx_);
    return regs_.holding_regs[addr];
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
    uint8_t buf[256];   // 与 ADU 上限一致（addr + func + data + crc2 = 256）
    while (running_.load())
    {
        uint64_t ts = 0;
        ssize_t n = port_.read(buf, sizeof(buf), 100, &ts);
        if (n <= 0) continue;

        parseFrame_(buf, (size_t)n, ts);
    }
}

// 解析一帧：先 CRC 校验，通过后组装 ModbusFrame，再按功能码把数据
// 组合成 16 位寄存器写入寄存器表。任何异常都直接丢弃，不污染寄存器表。
void SerialTask::parseFrame_(const uint8_t* buf, size_t len, uint64_t ts)
{
    if (len < 2) return;

    uint8_t addr    = buf[0];
    uint8_t funcode = buf[1];

    // 先确定本功能码下 CRC 覆盖长度与 CRC 在帧中的位置
    size_t crc_len = 0;   // CRC 覆盖字节数（addr .. data 区，不含 CRC 本身）
    size_t crc_pos = 0;   // CRC 低字节在 buf 中的下标

    if (funcode == 0x03 || funcode == 0x04) {
        // 读应答：addr + func + 字节数N + data[N] + crc(2)
        if (len < 3) return;
        uint8_t n = buf[2];
        crc_len   = 3 + n;
        crc_pos   = 3 + n;
    }
    else if (funcode == 0x06 || funcode == 0x10) {
        // 写应答：addr + func + 回显4字节 + crc(2)
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

    // ---- CRC 通过，组装通信层帧（不含 CRC）----
    ModbusFrame frame;
    frame.addr = addr;
    frame.func = funcode;
    frame.len  = (uint8_t)(crc_len - 2);          // 去掉 addr + func
    if (frame.len > sizeof(frame.data)) return;   // 理论上不会发生
    memcpy(frame.data, buf + 2, frame.len);

    std::lock_guard<std::mutex> lock(mtx_);

    if (funcode == 0x03 || funcode == 0x04) {
        // 读应答：frame.data = 字节数N + data[N]，无起始地址，靠 pending_ 定位
        uint8_t n = frame.data[0];
        if (!pending_.active || pending_.func != funcode) {
            std::cerr << "[SerialTask] 读应答与待处理请求不匹配，丢弃\n";
            return;
        }

        uint16_t start = pending_.start_addr;
        uint16_t nreg  = n / 2;                    // 寄存器个数
        if ((n & 1) || nreg != pending_.reg_count) {
            std::cerr << "[SerialTask] 读应答寄存器数量不符: 期望 " << pending_.reg_count
                      << "，实际 " << nreg << "，丢弃\n";
            return;
        }
        if ((size_t)start + nreg > SlaveRegisters::SIZE) {
            std::cerr << "[SerialTask] 寄存器地址越界，丢弃\n";
            return;
        }

        uint16_t* dst = (funcode == 0x04) ? regs_.input_regs : regs_.holding_regs;
        for (uint16_t i = 0; i < nreg; ++i)
            dst[start + i] = ((uint16_t)frame.data[1 + 2 * i] << 8) | frame.data[2 + 2 * i];  // 大端

        pending_.active = false;

        std::cout << "[RX] addr=" << (int)addr << " func=0x" << std::hex << (int)funcode
                  << std::dec << " start=" << start << " nreg=" << nreg
                  << " ts=" << ts << "ns:";
        for (uint16_t i = 0; i < nreg; ++i)
            std::cout << " " << std::hex << std::setw(4) << std::setfill('0') << dst[start + i];
        std::cout << std::dec << "\n";
    }
    else if (funcode == 0x06) {
        // 写单寄存器应答：回显 addr + func + reg_addr(2) + value(2)
        uint16_t reg = ((uint16_t)frame.data[0] << 8) | frame.data[1];
        uint16_t val = ((uint16_t)frame.data[2] << 8) | frame.data[3];
        if (reg < SlaveRegisters::SIZE) regs_.holding_regs[reg] = val;
        pending_.active = false;

        std::cout << "[RX] addr=" << (int)addr << " func=0x06 reg=" << reg
                  << " val=" << val << " ts=" << ts << "ns\n";
    }
    else { // 0x10
        // 写多寄存器应答：回显 addr + func + start(2) + count(2)，仅确认成功
        uint16_t start = ((uint16_t)frame.data[0] << 8) | frame.data[1];
        uint16_t count = ((uint16_t)frame.data[2] << 8) | frame.data[3];
        pending_.active = false;

        std::cout << "[RX] addr=" << (int)addr << " func=0x10 start=" << start
                  << " count=" << count << " ts=" << ts << "ns\n";
    }

    last_recv_ns_ = ts;
}
