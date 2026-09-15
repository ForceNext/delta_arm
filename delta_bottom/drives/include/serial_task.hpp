#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstdint>
#include "serial_port.hpp"
#include "Utilities/Timer.h"
#include "Utilities/PeriodicTask.h"
#include "yaml-cpp/yaml.h"

// ---------------------------------------------------------------------------
// 通信层：线上收到的一帧（CRC 已在 parseFrame_ 校验通过后丢弃，不再保留）
// data 存"功能码之后、CRC 之前"的内容，len 为其字节数
// ---------------------------------------------------------------------------
struct ModbusFrame
{
    uint8_t addr;          // 从站地址
    uint8_t func;          // 功能码
    uint8_t data[252];     // 数据区（ADU 上限 256 = addr + func + data + crc2）
    uint8_t len;           // data 实际字节数
};

// ---------------------------------------------------------------------------
// 业务层：从站寄存器镜像。Modbus 数据模型是 16 位寄存器（线上大端），
// 业务代码按寄存器地址取值，不关心字节流、字节序与 CRC。
// ---------------------------------------------------------------------------
struct SlaveRegisters
{
    static constexpr size_t SIZE = 256;

    uint16_t input_regs[SIZE];    // 输入寄存器 3x（0x04 读）
    uint16_t holding_regs[SIZE];  // 保持寄存器 4x（0x03 读 / 0x06 0x10 写）
};

// ---------------------------------------------------------------------------
// 主站事务上下文：读应答帧（0x03/0x04）不含起始地址，主站需自行记录
// "刚才发了什么请求"，收到应答时据此把数据定位到寄存器表。
// ---------------------------------------------------------------------------
struct PendingRequest
{
    bool     active     = false;
    uint8_t  func       = 0;
    uint16_t start_addr = 0;
    uint16_t reg_count  = 0;
};

class SerialTask
{
    private:
        char yaml_err = 0;

        // 接收线程配置（从 configs/thread_config.yaml 的 interface_task.task_thread 读取）
        int   cpu_      = 0;      // 绑定的 CPU 核
        int   priority_ = 0;      // 线程优先级（SCHED_FIFO）

        SerialPort& port_;                  // 串口（发送/接收共用，外部传入）
        std::thread recv_thread_;           // 接收线程句柄
        std::atomic<bool> running_{false};  // 线程运行标志

        // 寄存器镜像 + 事务上下文：接收线程写、业务线程读，用 mutex 保护
        SlaveRegisters regs_;
        PendingRequest pending_;
        mutable std::mutex mtx_;

        // 最近一帧成功接收的时间戳（单调时钟纳秒），用于测量响应延迟
        std::atomic<uint64_t> last_recv_ns_{0};

        void recvThread_();
        void parseFrame_(const uint8_t* buf, size_t len, uint64_t ts);

    public:
        explicit SerialTask(SerialPort& port);
        ~SerialTask();

        void start();                        // 启动接收线程
        void stop();                         // 停止并回收接收线程

        // 发送读/写请求前登记本次事务（主站一次只有一个未完成请求）
        void setPendingRequest(uint8_t func, uint16_t start_addr, uint16_t reg_count);

        // 按寄存器地址取值（未收到过的地址返回 0）
        uint16_t inputReg(uint16_t addr) const;
        uint16_t holdingReg(uint16_t addr) const;

        // 最近一帧成功接收的时间戳（单调时钟纳秒）
        uint64_t lastRecvNs() const { return last_recv_ns_.load(); }
};

#endif // SERIAL_HPP
