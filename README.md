# delta_arm

Delta 并联机械臂项目。当前已实现下位机 `delta_bottom`：在 Linux 上通过串口（RS485 / USB 转串口）以 **Modbus RTU** 协议控制 **3 台正点原子 PDxxS1 闭环步进电机驱动器**，并以 **200 Hz** 循环从共享内存读取上位机目标、下发电机绝对位置指令。

## 当前状态

**已实现（可运行）**：串口通信 → Modbus 组帧/校验 → 电机使能与绝对位置控制 → 200 Hz 控制循环 → 上下位机共享内存通信。

**未实现**：运动学正逆解（delta 逆解 / 正解）。当前阶段把上位机下发的 `x/y/z` **直接当作三个电机的目标角度**（单位：度）下发电机，用于跑通链路与演示。详细见 [后续规划](#后续规划路线图)。

## 关键设计

- **控制频率**：200 Hz（`clock_nanosleep` 绝对时间调度，避免周期漂移）
- **通信协议**：Modbus RTU（实际使用 `04H` 读输入寄存器 / `06H` 写单寄存器 / `10H` 写多寄存器）
- **驱动器**：正点原子 PDxxS1 闭环步进驱动器，位置分辨率 `51200` = 一圈（360°）
- **上下位机通信**：POSIX 共享内存（`shm_open` + `mmap`）+ System V 信号量互斥（key `0x5A11`）
- **启动探测**：逐个读 `0x2A`（实时位置）探测在线电机，只控制探测到的从站

## 架构与数据流

```
上位机(独立进程，不在本工程)          本工程 delta_bottom                    电机 ×3
─────────────────────────           ────────────────────────               ───────
写 ArmCommand{seq, mode, x, y, z}  200Hz 循环：                            PDxxS1
 x/y/z = 电机1/2/3 目标角(度)       ① 读命令（sem_p 加锁）
  ─────── 共享内存 /delta_arm ────▶ ② mode==1 时：角度(度) × DEG2CNT → 计数
                                    ③ moveAbsolute 逐台下发  ──Modbus RTU──▶
  ◀─────── 发布 ArmStatus ────────  ④ 发布状态（回显目标值，非实测）
```

分层调用链：

```
main.cpp（应用层：200 Hz 循环，读共享内存 → 角度换算 → 下发）
   │ MotorMapping::moveAbsolute() 等
   ▼
MotorMapping（kinematics/：角度↔计数换算 + 扫描/使能/绝对位置，组命令码）
   │ writeRegisters() / readInputRegisters()
   ▼
ModbusMaster（drives/：组帧 + CRC-16 + 发送 + 两段超时收应答 + 校验）
   │ readExact() / write() / flushInput()
   ▼
SerialPort（drives/：termios 串口，8N1 原始模式，半双工 tcdrain）
```

### 控制循环（[user/main.cpp](delta_bottom/user/main.cpp)）

```cpp
while (running) {
    // 1. 读上位机命令（共享内存，加锁）
    ArmCommand cmd;
    shm.readCommand(cmd);

    // 2. 角度(度) → 电机计数：51200 = 一圈
    double target[3] = {cmd.x, cmd.y, cmd.z};
    uint32_t counts[3];
    for (int i = 0; i < 3; ++i)
        counts[i] = (uint32_t)llround(target[i] * DEG2CNT);

    // 3. mode==1 时逐台下发绝对位置（失败累加错误计数）
    if (cmd.mode == 1)
        for (size_t i = 0; i < addrs.size() && i < 3; ++i)
            if (!MotorMapping::moveAbsolute(master, addrs[i], counts[i], 0, MOVE_ACCEL, MOVE_SPEED))
                ++err[i];

    // 4. 发布状态（当前回显目标值，未回读电机实际位置）
    shm.writeStatus(status);

    // 5. 睡到下一个 200 Hz 绝对周期边界
}
```

> **注意**：当前循环只**下发**指令并回显目标值，**没有回读电机实际位置/转速**。电机实际状态回读 + 正解发布属于后续规划（见 [路线图](#后续规划路线图)）。

## 模块划分

| 模块 | 目录 | 职责 | 状态 |
| --- | --- | --- | --- |
| `SerialPort` | `drives/` | 硬件层：termios 串口 8N1、`tcdrain` 半双工、`readExact` 按帧长精确收帧 | ✅ 已实现 |
| `ModbusMaster` | `drives/` | 协议层：组帧 + CRC + 同步事务（发→收→校验） | ✅ 已实现 |
| `MotorMapping` | `kinematics/` | 电机扫描/使能/运动控制 + 角度↔计数换算 | ✅ 已实现 |
| `Kinematics`（IK/FK） | `kinematics/` | delta 逆解 / 正解 | 🚧 空占位 |
| `arm_drive_task` | `task/` | 机械臂驱动任务 | 🚧 空占位 |
| `SharedMemory` / `sem_com` | `common/` | 上下位机共享内存 + System V 信号量互斥 | ✅ 已实现 |
| 控制循环 | `user/main.cpp` | 200 Hz 主循环，串联上述模块 | ✅ 已实现 |

## 项目结构

```
delta_arm/
├── README.md
└── delta_bottom/                     # 下位机工程（产物 delta_bottom 可执行文件）
    ├── build.sh                      # 一键编译脚本
    ├── CMakeLists.txt                # 顶层构建
    ├── configs/
    │   ├── hardware_config.yaml      # 串口 + 电机列表
    │   └── thread_config.yaml        # 遗留（原异步接收线程配置，现已不使用）
    ├── common/                       # 公共库 libcommon.a
    │   ├── include/
    │   │   ├── Utilities/SharedMemory.h   # 共享内存（shm_open + sem_com 锁）
    │   │   ├── Utilities/sem_com.h        # System V 信号量互斥锁
    │   │   ├── Utilities/*.h              # Timer/PeriodicTask/utilities 等
    │   │   └── cTypes.h / Types.h / cppTypes.h  # 遗留类型定义（源自 Cheetah 框架）
    │   └── src/Utilities/
    ├── drives/                       # 串口驱动库 libdrives.a
    │   ├── include/{serial_port, modbus_crc, modbus_master}.hpp
    │   └── src/*.cpp
    ├── kinematics/                   # 电机映射 + 正逆解库 libkinematics.a
    │   ├── include/{MotorMapping, Kinematics}.hpp
    │   └── src/{MotorMapping, Kinematics}.cpp
    ├── task/                         # 机械臂驱动任务库 libtask.a（占位）
    │   ├── include/arm_drive_task.hpp
    │   └── src/arm_drive_task.cpp
    ├── user/main.cpp                 # 应用层：200 Hz 控制循环
    ├── third_party/yaml-cpp/         # 配置解析（vendored）
    └── tools/
        ├── upper_sim.py              # Python 上位机模拟脚本（写角度，演示每秒一圈）
        ├── testpy.py                 # Python 调试脚本（读版本/位置/转速/状态）
        ├── 正点原子步进电机驱动器modbus-rtu控制协议V1.2(2).xlsx
        └── PDxxS1步进电机闭环驱动器用户手册_V1.0.pdf
```

## 通信协议

驱动器把 Modbus 的「寄存器起始地址」当作**命令码**使用（高字节补 0），读写均走标准 Modbus RTU 帧：

| 命令码 | 说明 |
| --- | --- |
| `0x20` | 读软硬件版本 |
| `0x29` | 读实时转速（int16，单位 RPM） |
| `0x2A` | 读实时位置（int32，`51200` = 一圈） |
| `0x2C` | 读运行状态 |
| `0x62` | 设置工作模式（0=通信位置 … 7=回零） |
| `0xF2` | 闭环绝对位置模式控制 |
| `0xF3` | 闭环相对位置模式控制 |
| `0xFA` | 使能控制（0=使能 / 1=失能） |
| `0xFB` | 清除状态（堵转/刹车/失能） |
| `0xFC` | 立即停止（刹车） |

闭环绝对位置（`0xF2`）数据区 8 字节 = 4 个寄存器：

| 字节 | 含义 |
| --- | --- |
| 1 | 电机旋转方向（0=正转 / 1=反转） |
| 2 | 加减速度（0~200，r/s²，0=直接启动） |
| 3~4 | 速度（0~6000 RPM，uint16） |
| 5~8 | 绝对位置（uint32，`51200` = 一圈） |

位置换算：`一圈 360° = 51200`，故 `90° = 12800`、`一圈/s = 51200 计数/s`。

## 运动参数与控制封装

`delta_bottom/user/main.cpp` 顶部集中定义运动参数：

```cpp
constexpr uint8_t  MOVE_ACCEL = 200;            // 加减速（0~200）
constexpr uint16_t MOVE_SPEED = 1000;           // 速度上限 RPM
constexpr double   DEG2CNT = 51200.0 / 360.0;   // 角度（度）→ 电机计数
```

`kinematics/MotorMapping` 封装了角度 ↔ 计数换算与运动控制（均为静态方法，`motorId` 直接对应从站地址 1~3）：

| 方法 | 功能 | 命令码 |
| --- | --- | --- |
| `loadMotorAddrs()` / `detectMotors()` | 读配置地址 / 探测在线电机 | `0x2A` |
| `setMotorEnable()` / `enableMotors()` | 单台 / 全部使能（失能） | `0xFA` |
| `SetMotorMode()` | 设置工作模式 | `0x62` |
| `moveAbsolute()` / `moveRelative()` | 绝对 / 相对位置控制 | `0xF2` / `0xF3` |
| `stopMotor()` / `clearStatus()` | 立即停止（刹车）/ 清除状态 | `0xFC` / `0xFB` |
| `angleToMotorPos()` / `motorPosToAngle()` | 关节角（度）↔ 电机计数换算 | — |

> 当前 200 Hz 循环只调用了 `detectMotors` / `enableMotors` / `moveAbsolute`；`SetMotorMode` / `moveRelative` / `stopMotor` / `clearStatus` 已封装但暂未在循环中调用。
>
> **注意**：`stopMotor()` 刹停后需及时调用 `clearStatus()` 清除状态，否则电机可能严重发烫（见手册 5.4.13）。

## 上下位机通信（共享内存）

上下位机**共用同一份头文件**（[common/include/Utilities/SharedMemory.h](delta_bottom/common/include/Utilities/SharedMemory.h)），保证内存布局一致：

```cpp
#define ARM_SHARED_MEMORY_NAME "/delta_arm"   // POSIX 共享内存名（须以 / 开头）
#define ARM_SEM_KEY            0x5A11         // System V 信号量 key（上下位机必须一致）

struct ArmCommand {          // 上位机 → 底层
    uint64_t seq = 0;        // 写入序号（每次写 +1）
    uint8_t  mode = 0;       // 0=空闲 1=位置模式
    double   x = 0.0;        // 电机 id1 目标角度（度）
    double   y = 0.0;        // 电机 id2 目标角度（度）
    double   z = 0.0;        // 电机 id3 目标角度（度）
};

struct ArmStatus {           // 底层 → 上位机
    uint64_t seq = 0;        // 写入序号
    uint8_t  state = 0;      // 0=空闲 1=运行
    uint8_t  online[3];      // 各电机在线标志
    double   theta[3];       // 实际下发的三个目标角（度）
    double   motor_pos[3];   // 实际下发的电机位置（计数）
    double   x, y, z;        // 回显坐标
};

struct ArmSharedData {       // 共享内存整体布局（锁不在块内）
    ArmCommand command;
    ArmStatus  status;
};
```

- **共享内存**：POSIX `shm_open` + `mmap`，名字 `/delta_arm`。
- **互斥锁**：独立的 System V 信号量（[common/include/Utilities/sem_com.h](delta_bottom/common/include/Utilities/sem_com.h)），key = `0x5A11`，不在共享内存块内部。
- **读写**：每次读/写 `command` 或 `status` 前 `sem_p()` 加锁、之后 `sem_v()` 解锁。
- **首次创建**：信号量用 `IPC_CREAT | IPC_EXCL` 判断是否首次创建——首次创建者初始化为 1，其余进程只打开不重置；`SEM_UNDO` 保证进程异常退出时自动释放持有的锁。

`ArmSharedMemory` 类封装了上述逻辑：

```cpp
ArmSharedMemory shm;           // 构造时打开信号量锁（key = ARM_SEM_KEY）
shm.connect();                 // 首次创建 / 其余附加共享内存

ArmCommand cmd;
shm.readCommand(cmd);          // 底层读命令（加锁 → 读 → 解锁）
ArmStatus st;
shm.writeStatus(st);           // 底层写状态（加锁 → 写 → 解锁）
```

### 上位机模拟脚本（`tools/upper_sim.py`）

真正的上位机就绪前，用 Python 脚本模拟上位机向共享内存写角度：

```bash
cd delta_bottom
python3 tools/upper_sim.py            # x 轴(id1)每秒 1 圈
python3 tools/upper_sim.py xyz        # x/y/z 三轴一起，每秒 1 圈
python3 tools/upper_sim.py xy 2.0     # x/y 两轴，每秒 2 圈
```

- 先启动底层 `./build/delta_bottom`，再运行该脚本。
- 脚本以 200 Hz 写入，角度按 `360 * rps / 200` 每拍累加；每秒打印各轴角度与底层回读的 `status.seq`（>0 表示链路已打通）。
- 退出（Ctrl-C）时自动写 `mode=0`（空闲），底层停止下发。
- 脚本用 `ctypes` 定义与 C++ 一致的 `ArmCommand/ArmStatus` 布局（共 128 字节），并用 System V 信号量（key `0x5A11`）加锁，与底层严格对齐。

## 构建与运行

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
    name: "motor_1"
    speed_rpm: 1000
    accel: 200
  - addr: 2
    name: "motor_2"
    speed_rpm: 1000
    accel: 200
  - addr: 3
    name: "motor_3"
    speed_rpm: 1000
    accel: 200
```

- `serial.port` / `serial.baudrate`：串口与波特率。
- `motors[].addr`：驱动器从站地址；程序启动时逐个探测，**只控制在线者**，未连接的地址不会拖慢循环。
- 目前 `loadMotorAddrs()` 只读取 `motors[].addr`，`name` / `speed_rpm` / `accel` 为预留字段，暂未在控制循环中使用。

## 硬件与依赖

- Linux（使用 `termios`、`select`、`clock_nanosleep`）
- C++17
- CMake ≥ 3.16
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)（vendored 于 `third_party/yaml-cpp`）
- [Eigen3](https://eigen.tuxfamily.org/)（`common/include/cppTypes.h` 等遗留类型定义依赖，当前 200 Hz 循环未直接使用）

## 调试

- `tools/testpy.py`：Python 脚本，逐个读版本 / 位置 / 转速 / 状态，用于验证接线与通信（无需编译 C++）。
- 运行时程序每 1 s 打印一次「实际频率」与各电机的「错误计数」，用于确认循环是否稳定在 200 Hz、通信是否有丢帧。

## 后续规划（路线图）

以下为**尚未实现**的部分，供后续开发参考：

1. **运动学正逆解**（[kinematics/src/Kinematics.cpp](delta_bottom/kinematics/src/Kinematics.cpp) 目前为空）：实现 delta 逆解 IK（末端 xyz → 三个关节角）与正解 FK，替换 `main.cpp` 里「xyz 直接当角度」的临时做法。
2. **电机计数映射**：关节角 → 电机计数需纳入减速比、零点偏移、转向（当前假设角度与计数线性 1:1）。
3. **状态回读与发布**：在控制循环内读回电机实际位置/转速，正解出实际末端坐标后发布，替代当前「回显目标值」。
4. **`task/arm_drive_task`**（[task/](delta_bottom/task/) 目前为空）：把驱动逻辑任务化，与规划/轨迹模块解耦。
5. **共享内存无锁化（可选）**：用 seqlock 替代 System V 信号量，读者永不阻塞，更适合 200 Hz 实时循环。
