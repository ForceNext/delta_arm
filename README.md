# delta_arm

Delta 并联机械臂项目。当前已实现下位机 `delta_bottom`：在 Linux 上通过串口（RS485 / USB 转串口）以 **Modbus RTU** 协议控制 **3 台正点原子 PDxxS1 闭环步进电机驱动器**，并以 **200 Hz** 循环从共享内存读取上位机目标、下发电机绝对位置指令。

## 当前状态

- **已实现（可运行）**：实时任务框架 `PeriodicTask`（`timerfd` 周期调度 + `SCHED_FIFO` 实时优先级 + CPU 亲和性绑定 + 线程生命周期管理）；串口驱动 `SerialPort`、Modbus 主站 `ModbusMaster`、电机控制封装 `MotorMapping`、上下位机共享内存 `ArmSharedMemory`，以及机械臂驱动任务 `ArmDriveTask`（读共享内存 → 角度换算 → `moveAbsolute` 下发 → 发布状态）。
- **控制链路已贯通**：主循环已从旧的 `clock_nanosleep` 单线程循环迁移到 `PeriodicTask` 框架。`ArmDriveTask::run()` 在 200 Hz 周期内完成「读 `ArmCommand` → 解析 xyz 目标角 → 角度×`DEG2CNT` 计数 → 逐台 `moveAbsolute` 下发 → 回写 `ArmStatus`」的端到端链路，串口 / Modbus / 电机控制均已接入。旧的注释版 `main()`（[user/main.cpp](delta_bottom/user/main.cpp) 顶部）已被取代，仅作历史参考。
- **未实现**：运动学正逆解（delta 逆解 / 正解）。当前阶段把上位机下发的 `x/y/z` **直接当作三个电机的目标角度**（单位：度）下发电机，用于跑通链路与演示。

## 关键设计

- **控制频率**：200 Hz（默认周期 `0.005 s`，由 `timerfd`（`CLOCK_MONOTONIC`）精确唤醒，避免周期漂移）
- **实时调度**：`SCHED_FIFO` + 可配置优先级（默认 75），线程绑定到指定 CPU；无 `root` / `CAP_SYS_NICE` 权限时**自动降级为普通调度**继续运行
- **通信协议**：Modbus RTU（实际使用 `04H` 读输入寄存器 / `06H` 写单寄存器 / `10H` 写多寄存器）
- **驱动器**：正点原子 PDxxS1 闭环步进驱动器，位置分辨率 `51200` = 一圈（360°）
- **上下位机通信**：POSIX 共享内存（`shm_open` + `mmap`）+ System V 信号量互斥（key `0x5A11`）
- **启动探测**：逐个读 `0x2A`（实时位置）探测在线电机，只控制探测到的从站

## 架构与数据流

### 控制链路

```
上位机(独立进程，不在本工程)          本工程 delta_bottom                    电机 ×3
─────────────────────────           ────────────────────────               ───────
写 ArmCommand{seq, mode, x, y, z}  ArmDriveTask 200Hz 循环：             PDxxS1
 x/y/z = 电机1/2/3 目标角(度)       ① 读命令（sem_p 加锁）
  ─────── 共享内存 /delta_arm ────▶ ② mode==1 时：角度(度) × DEG2CNT → 计数
                                    ③ moveAbsolute 逐台下发  ──Modbus RTU──▶
  ◀─────── 发布 ArmStatus ────────  ④ 发布状态（回显目标值，非实测）
```

> `ArmDriveTask` 在 `init()` 中完成共享内存连接与电机探测/使能，之后 `run()` 按 200 Hz 循环执行上图的 ①②③④。

### 分层调用链

```
main.cpp（应用层：读 thread_config → 建 ArmDriveTask → 绑定 CPU/优先级 → start）
   │ ArmDriveTask::run()（读共享内存 → 角度换算 → moveAbsolute → 发布状态）
   ▼
MotorMapping（kinematics/：角度↔计数换算 + 扫描/使能/绝对位置，组命令码）
   │ writeRegisters() / readInputRegisters()
   ▼
ModbusMaster（drives/：组帧 + CRC-16 + 发送 + 两段超时收应答 + 校验）
   │ readExact() / write() / flushInput()
   ▼
SerialPort（drives/：termios 串口，8N1 原始模式，半双工 tcdrain）
```

## 实时任务框架（`PeriodicTask`）

`common/Utilities/PeriodicTask` 是本工程的控制线程基础设施：

- **周期调度**：每个任务线程内部用 `timerfd` 按 `period` 精确唤醒（`func()` 中 `read(timerFd)` 阻塞到下一拍）。
- **实时优先级**：`start()` 里通过 `pthread_attr_*` 设置 `SCHED_FIFO`；若 `pthread_create` 返回 `EPERM`（无 `root` / `CAP_SYS_NICE`），自动用默认属性重试，降级为普通调度。
- **CPU 亲和性**：`set_bind_cpu(id)` 把线程绑到指定核（`pthread_setaffinity_np`）。
- **生命周期**：`start()` 异步启动线程；`stop()` 置 `_running=false` → `pthread_join` 收尾 → 调用 `cleanup()`。
- **纯虚接口**：子类实现 `init()` / `run()` / `cleanup()`。

关键 API：

| 成员 | 说明 |
| --- | --- |
| `PeriodicTask(manager, period, name)` | 构造并注册到 `PeriodicTaskManager` |
| `start()` / `stop()` | 启动 / 停止线程 |
| `set_bind_cpu(int id)` | 绑定 CPU |
| `set_sched_priority(int p)` | 设置 `SCHED_FIFO` 优先级 |
| `init()` / `run()` / `cleanup()` | 纯虚，子类实现 |
| `PeriodicTaskManager::get_instance()` | 单例任务管理器 |
| `addTask()` / `stopAll()` / `printStatus()` | 管理任务 / 全停 / 打印各任务周期统计 |

另提供模板 `PeriodicFunction`（包装自由函数）、`PeriodicMemberFunction<T>`（包装成员函数）与 `PrintTaskStatus`（周期打印任务状态）。

## 模块划分

| 模块 | 目录 | 职责 | 状态 |
| --- | --- | --- | --- |
| `PeriodicTask` / `PeriodicTaskManager` | `common/` | 实时任务框架：周期调度 + 实时优先级 + CPU 亲和性 + 线程生命周期 | ✅ 已实现 |
| `SerialPort` | `drives/` | 硬件层：termios 串口 8N1、`tcdrain` 半双工、`readExact` 按帧长精确收帧 | ✅ 已实现 |
| `ModbusMaster` | `drives/` | 协议层：组帧 + CRC + 同步事务（发→收→校验） | ✅ 已实现 |
| `MotorMapping` | `kinematics/` | 电机扫描/使能/运动控制 + 角度↔计数换算 | ✅ 已实现 |
| `Kinematics`（IK/FK） | `kinematics/` | delta 逆解 / 正解 | 🚧 空占位 |
| `arm_drive_task` | `task/` | 机械臂驱动任务（`PeriodicTask` 子类）：读共享内存 → 角度换算 → `moveAbsolute` 下发 → 发布状态 | ✅ 已实现 |
| `SharedMemory` / `sem_com` | `common/` | 上下位机共享内存 + System V 信号量互斥 | ✅ 已实现 |
| 应用入口 | `user/main.cpp` | 读线程配置 → 建任务 → 保活 → 优雅退出 | ✅ 已实现 |

## 项目结构

```
delta_arm/
├── README.md
└── delta_bottom/                     # 下位机工程（产物 delta_bottom 可执行文件）
    ├── build.sh                      # 一键编译脚本
    ├── CMakeLists.txt                # 顶层构建
    ├── configs/
    │   ├── hardware_config.yaml      # 串口 + 电机列表（SerialPort/MotorMapping 使用）
    │   └── thread_config.yaml        # 线程配置：周期 / 绑定 CPU / 实时优先级
    ├── common/                       # 公共库 libcommon.a
    │   ├── include/
    │   │   ├── Utilities/SharedMemory.h   # 共享内存（shm_open + sem_com 锁）
    │   │   ├── Utilities/PeriodicTask.h   # 实时任务框架
    │   │   ├── Utilities/sem_com.h        # System V 信号量互斥锁
    │   │   └── Utilities/*.h              # Timer/utilities/Utilities_print 等
    │   └── src/Utilities/
    ├── drives/                       # 串口驱动库 libdrives.a
    │   ├── include/{serial_port, modbus_crc, modbus_master}.hpp
    │   └── src/*.cpp
    ├── kinematics/                   # 电机映射 + 正逆解库 libkinematics.a
    │   ├── include/{MotorMapping, Kinematics}.hpp
    │   └── src/{MotorMapping, Kinematics}.cpp
    ├── task/                         # 机械臂驱动任务库 libtask.a（ArmDriveTask 控制循环）
    │   ├── include/arm_drive_task.hpp
    │   └── src/arm_drive_task.cpp
    ├── user/main.cpp                 # 应用入口：读线程配置 → 建任务 → 保活退出
    ├── third_party/yaml-cpp/         # 配置解析（vendored）
    └── tools/
        ├── set_angle.py              # Python 脚本：输入角度 → 写共享内存 → 电机转到对应角度
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

`kinematics/MotorMapping` 封装了角度 ↔ 计数换算与运动控制（均为静态方法，`motorId` 直接对应从站地址 1~3）：

| 方法 | 功能 | 命令码 |
| --- | --- | --- |
| `loadMotorAddrs()` / `detectMotors()` | 读配置地址 / 探测在线电机 | `0x2A` |
| `setMotorEnable()` / `enableMotors()` | 单台 / 全部使能（失能） | `0xFA` |
| `SetMotorMode()` | 设置工作模式 | `0x62` |
| `moveAbsolute()` / `moveRelative()` | 绝对 / 相对位置控制 | `0xF2` / `0xF3` |
| `stopMotor()` / `clearStatus()` | 立即停止（刹车）/ 清除状态 | `0xFC` / `0xFB` |
| `angleToMotorPos()` / `motorPosToAngle()` | 关节角（度）↔ 电机计数换算 | — |

> 运动参数（加减速、速度）通过 `moveAbsolute()` / `moveRelative()` 的默认参数传入（`accel=200`、`speed=1000 RPM`）。
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

### 上位机指令脚本（`tools/set_angle.py`）

真正的上位机就绪前，用 Python 脚本向共享内存发布目标角度：

```bash
cd delta_bottom
python3 tools/set_angle.py 30         # 三轴 x/y/z 都转到 30°
python3 tools/set_angle.py 30 x       # 只有 x（id1）转到 30°，其余轴保持原值
python3 tools/set_angle.py            # 不带参数则交互式输入角度
```

- 先启动底层 `./build/delta_bottom`，再运行该脚本。
- 脚本用 `ctypes` 定义与 C++ 一致的 `ArmCommand/ArmStatus` 布局（共 128 字节），并用 System V 信号量（key `0x5A11`）加锁，与底层严格对齐。
- 写入 `mode=1`、`seq+1`，仅更新指定轴的角度，其余轴保持原值；随后回显底层状态（`state`、`seq`）便于确认链路是否打通。

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
./build/delta_bottom                      # 使用 configs/thread_config.yaml
./build/delta_bottom 路径/到/线程配置.yaml   # 指定线程配置文件
```

Ctrl-C（或 `SIGTERM`）触发优雅退出：停止任务线程并执行 `cleanup()`。

> **实时调度需要权限**：`SCHED_FIFO` 优先级调度需要 `root` 或 `CAP_SYS_NICE`。
> - `sudo ./build/delta_bottom`，或
> - `sudo setcap cap_sys_nice+ep ./build/delta_bottom`（授权一次，之后普通用户即可）
>
> 无权限时程序会**自动降级为普通调度**继续运行（黄色告警），周期精度靠 `timerfd` 仍能保持，只是没有实时保证。
>
> 另外构造函数会写 `/dev/cpu_dma_latency`（关闭 CPU 深度休眠、降低延迟），该项同样需要 `root`；失败仅打印 error，不影响运行。

## 配置

`delta_bottom/configs/thread_config.yaml`（任务线程配置，当前由 `main.cpp` 读取）：

```yaml
interface_task:
  task_thread:
    thread_t: 0.005        # 周期（秒），0.005 = 200 Hz
    bind_cpu: 4            # 绑定到的 CPU 核
    thread_priority: 75    # SCHED_FIFO 实时优先级
```

`delta_bottom/configs/hardware_config.yaml`（串口 + 电机列表，由 `SerialPort` / `MotorMapping` 读取）：

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

- Linux（使用 `termios`、`select`、`timerfd`、`pthread` 实时调度）
- C++17
- CMake ≥ 3.16
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)（vendored 于 `third_party/yaml-cpp`）
- [Eigen3](https://eigen.tuxfamily.org/)（`common/include/cppTypes.h` 等遗留类型定义依赖，当前控制循环未直接使用）

## 调试

- `tools/testpy.py`：Python 脚本，逐个读版本 / 位置 / 转速 / 状态，用于验证接线与通信（无需编译 C++）。
- 运行时 `ArmDriveTask` 每 1000 次循环打印一次实际耗时与频率（约每 5 s 一行），用于确认循环稳定在 200 Hz。
- `PeriodicTaskManager::printStatus()` / `printStatusOfSlowTasks()` 可打印各任务的最短/最长运行时长与周期，用于定位超时任务。

## 后续规划（路线图）

以下为**尚未实现**的部分，供后续开发参考：

1. **运动学正逆解**（[kinematics/src/Kinematics.cpp](delta_bottom/kinematics/src/Kinematics.cpp) 目前为空）：实现 delta 逆解 IK（末端 xyz → 三个关节角）与正解 FK，替换「xyz 直接当角度」的临时做法。
2. **电机计数映射**：关节角 → 电机计数需纳入减速比、零点偏移、转向（当前假设角度与计数线性 1:1）。
3. **状态回读与发布**：在控制循环内读回电机实际位置/转速，正解出实际末端坐标后发布，替代当前「回显目标值」。
4. **共享内存无锁化（可选）**：用 seqlock 替代 System V 信号量，读者永不阻塞，更适合 200 Hz 实时循环。
