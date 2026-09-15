#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <iostream>
#include <thread>
#include <atomic>
#include "serial_port.hpp"
#include "Utilities/Timer.h"
#include "Utilities/PeriodicTask.h"
#include "yaml-cpp/yaml.h"

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

        char yaml_err = 0;

        // 接收线程配置（从 configs/thread_config.yaml 的 interface_task.task_thread 读取）
        int   cpu_      = 0;      // 绑定的 CPU 核
        int   priority_ = 0;      // 线程优先级（SCHED_FIFO）
        float thread_t_ = 0.0f;   // 线程周期（秒），事件驱动模式下未使用

        SerialPort& port_;                  // 串口（发送/接收共用，外部传入）
        std::thread recv_thread_;           // 接收线程句柄
        std::atomic<bool> running_{false};  // 线程运行标志

        SERIAL_OBJ_ recv_obj_;              // 最近一帧解析结果

        void recvThread_();                 // 接收线程体（事件驱动循环）
        void parseFrame_(const uint8_t* buf, size_t len, uint64_t ts);  // 解析一帧到 recv_obj_

    public:
        explicit SerialTask(SerialPort& port);
        ~SerialTask();

        void start();                        // 启动接收线程
        void stop();                         // 停止并回收接收线程
};

#endif // SERIAL_HPP
