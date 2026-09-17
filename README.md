# delta_arm

Delta 并联机械臂项目。目前实现下位机 `delta_bottom`，通过串口（RS485 / USB 转串口）以 Modbus RTU 协议控制正点原子 PDxxS1 闭环步进电机驱动器。

## 关键设计

- **控制频率**：200 Hz
- **通信协议**：Modbus RTU（`04H` 读输入寄存器 / `06H` 写单寄存器 / `10H` 写多寄存器）
- **驱动器**：正点原子 PDxxS1 闭环步进驱动器（位置分辨率 `51200` = 一圈）

## 项目结构

```
delta_arm/
├── README.md
└── delta_bottom/                # 下位机
    ├── build.sh                 # 一键编译脚本
    ├── CMakeLists.txt           # 顶层构建
    ├── configs/
    │   ├── hardware_config.yaml # 串口 + 电机列表
    │   └── thread_config.yaml   # 遗留（原异步接收线程配置，现已不再使用）
    ├── drives/                  # 串口驱动库（静态库 libdrives.a）
    │   ├── include/
    │   │   ├── serial_port.hpp  # SerialPort：硬件层（termios 串口）
    │   │   ├── modbus_crc.hpp   # CRC-16/MODBUS
    │   │   └── modbus_master.hpp# ModbusMaster：同步事务层
    │   └── src/
    │       ├── serial_port.cpp
    │       └── modbus_master.cpp
    ├── user/
    │   └── main.cpp             # 应用层：200 Hz 控制循环
    ├── third_party/
    │   ├── yaml-cpp/            # 配置解析
    │   └── common/              # PeriodicTask 等（遗留，现不直接使用）
    └── tools/
        ├── testpy.py            # Python 调试脚本（读版本/位置/转速等）
        ├── 正点原子步进电机驱动器modbus-rtu控制协议V1.2(2).xlsx
        └── PDxxS1步进电机闭环驱动器用户手册_V1.0.pdf
```

## 功能

- Modbus RTU 主站（同步事务：组帧 → CRC → 发送 → 按长度收应答 → 校验）
- 200 Hz 控制循环，逐台向在线电机下发**闭环绝对位置**命令
- 启动时自动探测在线的电机，只控制探测到的从站
- 支持读实时位置 / 转速 / 状态等遥测

## 硬件与依赖

- Linux（使用 `termios`、`select`、`clock_nanosleep`）
- C++17
- CMake ≥ 3.16
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)（vendored 于 `third_party/yaml-cpp`）
- [Eigen3](https://eigen.tuxfamily.org/)（`third_party/common` 的依赖）

## 快速开始

### 构建

```bash
cd delta_bottom
./build.sh              # Release 编译
./build.sh --debug      # Debug 编译
./build.sh --clean      # 清空 build 目录后重编译
./build.sh --run        # 编译完成后直接运行
```

产物：`delta_bottom/build/delta_bottom`

### 运行

```bash
cd delta_bottom
./build/delta_bottom                     # 使用 configs/hardware_config.yaml
./build/delta_bottom 路径/到/配置.yaml    # 指定配置文件
```

Ctrl-C 退出（不刹车，电机停在当前位置）。

> 串口设备（如 `/dev/ttyACM0`）默认属于 `dialout` 组，运行前确保当前用户在组内。

## 配置

`delta_bottom/configs/hardware_config.yaml`：

```yaml
serial:
  port: "/dev/ttyACM0"     # 串口设备
  baudrate: 921600         # 波特率

motors:                    # 电机（从站）列表
  - addr: 1                # 从站地址（0=广播，1~255 从站）
    name: "joint_base"
    speed_rpm: 500
    accel: 50
    limit_min: -512000
    limit_max: 512000
  # ... 其余关节同理
```

- `serial.port` / `serial.baudrate`：串口与波特率。
- `motors[].addr`：驱动器从站地址；程序启动时逐个探测，**只控制在线者**，未连接的地址不会拖慢循环。

## 架构与数据流

```
main.cpp（应用层：200 Hz 控制循环 + 命令码封装）
   │ writeRegisters() / readInputRegisters()
   ▼
ModbusMaster（通信层：组帧 + CRC + 发送 + 两段超时收应答 + 校验）
   │ readExact() / write() / flushInput()
   ▼
SerialPort（硬件层：termios 串口，8N1 原始模式，半双工 tcdrain）
```

- **SerialPort**：封装 Linux 串口；`write()` 末尾 `tcdrain` 保证半双工发完再等应答；`readExact()` 按期望长度精确收帧，避免帧尾静默等待拖慢循环。
- **ModbusMaster**：同步事务，一次调用 = 一次「发 + 收 + 校验」。提供读（0x03/0x04）、写单（0x06）、写多（0x10）。
- **main**：200 Hz 循环，用绝对时间（`clock_nanosleep(TIMER_ABSTIME)`）调度避免周期漂移；逐台下发闭环绝对位置，失败跳过并计数。

## 通信协议

驱动器把 Modbus 的「寄存器起始地址」当作**命令码**使用（高字节补 0），读写均走标准 Modbus RTU 帧：

| 命令码 | 说明 |
| --- | --- |
| `0x20` | 读软硬件版本 |
| `0x29` | 读实时转速（int16，单位 RPM） |
| `0x2A` | 读实时位置（int32，`51200` = 一圈） |
| `0x2C` | 读运行状态 |
| `0xF2` | 闭环绝对位置模式控制 |
| `0xFA` | 使能控制（0=使能 / 1=失能） |
| `0xFC` | 立即停止（刹车） |

闭环绝对位置（`0xF2`）数据区 8 字节 = 4 个寄存器：

| 字节 | 含义 |
| --- | --- |
| 1 | 电机旋转方向（0=正转 / 1=反转） |
| 2 | 加减速度（0~200，r/s²，0=直接启动） |
| 3~4 | 速度（0~6000 RPM，uint16） |
| 5~8 | 绝对位置（uint32，`51200` = 一圈） |

位置换算：`一圈 360° = 51200`，故 `90° = 12800`、`一圈/s = 51200 计数/s`。

## 运动参数

`delta_bottom/user/main.cpp` 顶部集中定义：

```cpp
constexpr uint8_t  MOVE_ACCEL = 200;   // 加减速
constexpr uint16_t MOVE_SPEED = 1000;  // 速度上限 RPM
constexpr double   REV = 51200.0;      // 一圈对应的计数
```

当前演示轨迹为「一圈/s 匀速旋转」，位置按真实时间 `pos = REV * t` 计算；接正逆解 / 轨迹规划时替换这段即可。

## 调试

- `tools/testpy.py`：Python 脚本，逐个读版本 / 位置 / 转速 / 状态，用于验证接线与通信（无需编译 C++）。
- 运行时程序每 1 s 打印一次「实际频率」与各电机的「实际转速 / 错误计数」，用于确认循环是否稳定在 200 Hz、通信是否有丢帧。
