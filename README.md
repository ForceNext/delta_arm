# delta_arm — Delta 并联机械臂

Linux 下用 **Modbus RTU**（RS485 / USB 转串口）控制 3 台 **正点原子 PDxxS1 闭环步进驱动器**。
上位机通过**共享内存**与底层交换数据，**坐标正逆解在上位机**，本工程只负责「角度 ↔ 计数器 ↔ 总线」。

---

## 当前状态

| 部分 | 状态 |
| --- | --- |
| 实时任务框架 `PeriodicTask`（timerfd 周期 + `SCHED_FIFO` + CPU 亲和） | ✅ 可用 |
| 串口 `SerialPort`（任意波特率、半双工收帧、帧间静默） | ✅ 可用 |
| Modbus 主站 `ModbusMaster`（组帧 + CRC + 事务 + 失败原因返回码） | ✅ 可用 |
| 电机封装 `MotorMapping`（探测 / 初始化 / 使能 / 下发 / 回读 / 角度换算 / 减速比） | ✅ 可用 |
| 共享内存 `ArmSharedMemory`（128 字节，启动清零） | ✅ 可用 |
| 控制循环 `ArmDriveTask::run()` | ✅ 读共享内存三个角度，**每拍只给一台电机发绝对位置**（1→2→3 轮转，每台约 66 Hz） |

> `theta[3]` 按**关节角**下发：`计数 = θ × 减速比 × 51200/360`（减速比取 `hardware_config.yaml` 的 `joint.gear_ratio`）。
> 若上位机给的是电机角，把 `arm_drive_task.cpp` 里的 `THETA_IS_JOINT_ANGLE` 改成 `false`。

## 目录结构

```
delta_arm/
├── README.md
└── delta_bottom/                     # 下位机工程（产物 build/delta_bottom）
    ├── build.sh                      # 一键编译：./build.sh [--clean|--debug|--run]
    ├── CMakeLists.txt                # 顶层构建（common / drives / user/task）
    ├── configs/
    │   ├── hardware_config.yaml      # 串口 + 电机地址 + 减速比
    │   └── thread_config.yaml        # 线程：周期 / 绑定 CPU / 实时优先级
    ├── common/                       # libcommon.a：PeriodicTask、SharedMemory、Timer 等
    ├── drives/                       # libdrives.a：serial_port、modbus_*、MotorMapping
    ├── user/
    │   ├── main.cpp                  # 入口：读线程配置 → 建任务 → 保活 → 优雅退出
    │   └── task/                     # libtask.a：ArmDriveTask（骨架）
    └── tools/                        # Python 脚本 + docs/（驱动器手册、协议表、机构简图）
```

## 共享内存契约（上位机 ↔ 底层）

POSIX 共享内存 `/delta_arm` + System V 信号量（key `0x5A11`），共 **128 字节**：

```c
struct ArmCommand {          // 上位机 → 底层
    uint64_t seq;            // 写入序号（每次写 +1）
    uint8_t  mode;           // 0=空闲  1=运行
    uint8_t  reserved[3];    // 对齐填充
    uint32_t magic;          // 布局魔数 0x444C5431（'DLT1'）
    double   theta[3];       // 三个角度（度）
};

struct ArmStatus {           // 底层 → 上位机
    uint64_t seq;            // 写入序号（= 控制循环计数）
    uint8_t  state;          // bit0=正在按命令运行；bit1=本次数据来自实测
    uint8_t  online[3];      // 各电机在线标志
    double   theta[3];       // 三个电机角（度）
    double   motor_pos[3];   // 三个电机位置（计数，51200 = 一圈）
    double   joint_theta[3]; // 三个关节角（度）= 电机角 ÷ 减速比
};
```

- **每次启动整块清零**（`mode=0`）：所以顺序是「先启动底层，再让上位机发命令」，残留的旧命令不会被执行。
- `magic` 不符（旧布局、或别的程序留下的同尺寸内存）也会被清零，避免按新字段误读旧数据。
- `theta[3]` 是**关节角**（会乘 `joint.gear_ratio` 再换算计数）；`tools/set_angle.py --unit` 默认 `joint`，两者必须一致。

## 构建与运行

```bash
cd delta_bottom
./build.sh                 # 编译（--clean 清空重编，--debug Debug，--run 编译后直接跑）
sudo ./build/delta_bottom  # 要实时优先级和 /dev/cpu_dma_latency 时需要 sudo；没权限也能跑（自动降级）
```

- 启动会**打开 `/dev/ttyACM0` 并清零共享内存**；运行期间串口被占用，别同时跑工具脚本或第二个实例。
- `argv[1]` 目前同时被当作线程配置和硬件配置路径（**已知问题**）；不加参数最省事。

## 配置

| 文件 | 关键项 |
| --- | --- |
| `configs/thread_config.yaml` | `thread_t`（周期秒，当前 0.005 = 200 Hz）、`bind_cpu`、`thread_priority` |
| `configs/hardware_config.yaml` | `serial`（端口 / 波特率 256000）、`motors`（从站地址）、`joint.gear_ratio`（三臂减速比） |

## 工具（`tools/`）

| 脚本 | 用途 |
| --- | --- |
| `probe_bus.py` | 总线探测/排障：地址扫描、成功率、只听不发、`--raw` 注入任意帧（内置模式只发 `04H` 读，不会让电机动作） |
| `read_angles.py` | **只读**打印三台角度；自动选数据源（底层在跑读共享内存，没跑则直连串口），避免两个主站抢总线；`--capture/--ref` 可显示相对角度 |
| `set_angle.py` | 向共享内存写三个角度（`--idle` 置空闲）；带"不拿陈旧/零基准"的护栏 |
| `testpy.py` | 串口直连调试（读版本/位置/转速/状态），参数写死，需要 `pyserial` |
| `docs/` | PDxxS1 用户手册、Modbus 协议表、机构简图 |

```bash
cd delta_bottom
python3 tools/probe_bus.py --baud 256000 --addr 1-3 --trials 5   # 三台是否都在线
python3 tools/read_angles.py                                     # 当前角度
python3 tools/set_angle.py --idle                                # 置空闲（不发角度）
python3 tools/set_angle.py 10 10 10                              # 发布三个角度
```

## 用到的 Modbus 寄存器

功能码：`04H` 读输入寄存器、`06H` 写单寄存器、`10H` 写多寄存器（自定义协议的功能码放在寄存器地址里）。

| 寄存器 | 作用 | 寄存器 | 作用 |
| --- | --- | --- | --- |
| `0x20` | 读软硬件版本 | `0x65` | 设置细分（=16） |
| `0x23 / 0x24` | 相电流 / 总线电压 | `0x67` | 设置串口波特率 |
| `0x29 / 0x2A / 0x2B` | 转速 / **位置** / 位置误差 | `0x69` | 设置协议（=Modbus） |
| `0x2C / 0x2D / 0x2E` | 运行状态 / 堵转标志 / 堵转电流 | `0xF2 / 0xF3` | 绝对 / 相对位置控制 |
| `0x30 / 0x32` | 到位标志 / 读驱动参数 | `0xF8 / 0xFA` | 位置清零 / 电机使能 |
| `0x60 / 0x62` | 从站地址 / 工作模式 | `0xFB / 0xFC` | 清除状态 / 立即停止（刹车） |

- 手册 5.1.3 时序要求：**两条完整指令间隔至少 2 ms**，且整帧必须连续发送。
- 实测一次事务约 **2.5~3.2 ms**（256000 bps 下线路传输只占约 0.7 ms，其余是驱动器响应延迟）；因此一个周期内只放得下一次事务。

## 首次使用：零点标定（必做）

驱动器返回的是**它自己的绝对计数**，和「θ=0 = 上臂水平」没有关系。不标定就发绝对角度，机械臂会大幅乱动。

1. 人工把机械臂摆到参考位形（例如三个上臂水平）；
2. 三台各发一帧位置清零 `0xF8`（`06H` 写 `0x00F8` = 1）：

   ```
   01 06 00 F8 00 01 C9 FB
   02 06 00 F8 00 01 C9 C8
   03 06 00 F8 00 01 C8 19
   ```

3. 用 `read_angles.py` 确认三台都变成 0 附近。

## 已知问题（实测）

1. **1 号（偶尔 3 号）驱动器会额外吐帧**：对一次读请求有时回「数据帧 + 异常帧 `01 84 04` + 异常帧 `01 84 01`」，三个帧 CRC 都正确；2 号从未出现。后果是回读可能被判成"从站异常码 0x04"。疑似物理层问题，建议先查**共地、120Ω 终端电阻、A/B 双绞与屏蔽**，再逐台隔离验证。
2. **停机后角度会缓慢漂移**（实测 2、3 号各下沉 2~3° 电机角）：停机后并没有刚性保持位置，需要保持时得显式发刹车/使能。
3. **`argv[1]` 双重语义**：线程配置与硬件配置共用一个路径参数。
4. **控制循环还很简**：`ArmStatus` 只发布了 `seq`（每拍递增，供工具判活）、`state`、`online[]`；**还没有回读**，所以 `theta/motor_pos/joint_theta` 恒为 0、`state` 的 bit1 也不置位。没有掉线保护；单次下发失败只在日志里提示一行，靠下一拍轮转重发收敛。

## `run()` 当前逻辑

1. 读共享内存 `ArmCommand`（三个目标角度 + `mode`），并发布 `ArmStatus`（`seq` 每拍递增，供工具判活）；
2. 首次下发前 `ensureDrivesReady()`：每 100 ms 轮转探测一台 → 三台在线后逐台 `initMotor`（协议 Modbus + 细分 16）→ `enableMotors`；
3. **在线判定**：每台"连续无应答"达到 **10 次**（约 150 ms）时，单独探一次该电机
   - 探测有应答 → 仍按在线处理（清零计数）；探测无应答 → 判定掉线，`online[i]=0`，跳过该轴；
   - 任何"有应答"的结果（成功 / CRC 错 / 从站异常码）都会清零连续无应答计数——只有真正收不到任何字节才算超时；
4. **掉线恢复**：已掉线的电机每 **1 s** 轮转重探一台，探测到就置回在线并自动重新纳入下发；
5. `mode != 1` 视为空闲，不下发；
6. 每个周期只给**一台**电机发 `moveAbsoluteRc`：角度 → 电机计数 → 绝对位置，`1→2→3` 轮转；
7. 下发失败只在"原因变化"时打印一行（否则会刷屏），下一拍轮转时自动重发。

待补充：回读实测位置并发布 `ArmStatus`（`theta/motor_pos/joint_theta` 现在恒为 0）、`0xF9/0xFB` 错误状态清除与校验。
