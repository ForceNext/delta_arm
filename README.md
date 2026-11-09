# delta_arm — Delta 并联机械臂

Linux 下用 **Modbus RTU**（RS485 / USB 转串口）控制 3 台 **正点原子 PDxxS1 闭环步进驱动器**。
上位机通过**共享内存**下发三个**关节角**，底层负责「关节角 → 电机计数 → 总线下发」。
**坐标正逆解在上位机**，本工程不含 IK/FK。

---

## 当前状态

| 部分 | 状态 |
| --- | --- |
| 实时任务框架 `PeriodicTask`（timerfd 周期 + `SCHED_FIFO` + CPU 亲和） | ✅ 可用 |
| 串口 `SerialPort`（任意波特率、半双工收帧、帧间静默） | ✅ 可用 |
| Modbus 主站 `ModbusMaster`（组帧 + CRC + 事务 + 失败原因返回码） | ✅ 可用 |
| 电机封装 `MotorMapping`（探测 / 初始化 / 使能 / 下发 / 回读 / 角度换算 / 减速比 / 限幅 / 方向） | ✅ 可用 |
| 共享内存 `ArmSharedMemory`（128 字节，启动清零） | ✅ 可用 |
| 控制循环 `ArmDriveTask::run()` | ✅ 每拍只给**一台**电机发绝对位置（1→2→3 轮转） |
| 关节角限幅 / 方向系数 / 速度加减速 | ✅ 全部读配置 |
| 在线判定（连续 10 次超时→专门探测）与掉线恢复（每秒重探） | ✅ 可用 |
| 实测位置回读（`ArmStatus.theta/motor_pos/joint_theta`） | ⬜ **未实现**（现在恒为 0） |

## 目录结构

```
delta_arm/
├── README.md
└── delta_bottom/                     # 下位机工程（产物 build/delta_bottom）
    ├── build.sh                      # 一键编译：./build.sh [--clean|--debug|--run]
    ├── CMakeLists.txt                # 顶层构建（common / drives / user/task）
    ├── configs/
    │   ├── hardware_config.yaml      # 串口 + 电机（地址/细分/速度/加减速）+ 关节（减速比/方向/限幅）
    │   └── thread_config.yaml        # 线程：周期 / 绑定 CPU / 实时优先级
    ├── common/                       # libcommon.a：PeriodicTask、SharedMemory、Timer 等
    ├── drives/                       # libdrives.a：serial_port、modbus_*、MotorMapping
    ├── user/
    │   ├── main.cpp                  # 入口：读线程配置 → 建任务 → 保活 → 优雅退出
    │   └── task/                     # libtask.a：ArmDriveTask（控制循环）
    └── tools/                        # Python 脚本 + docs/（驱动器手册、协议表、机构简图）
```

## 构建与运行

```bash
cd delta_bottom
./build.sh                 # 编译（--clean 清空重编，--debug Debug，--run 编译后直接跑）
sudo ./build/delta_bottom  # 实时优先级与 /dev/cpu_dma_latency 需要 root；没权限会自动降级
```

- 启动会**打开 `/dev/ttyACM0`、清零共享内存**，然后开始探测电机；运行期间串口被占用，**不要再跑工具脚本或第二个实例**。
- `argv[1]` 同时被当作线程配置和硬件配置路径（已知问题），不加参数最省事。

## 配置（`configs/hardware_config.yaml`）

| 字段 | 含义 |
| --- | --- |
| `serial.port` / `serial.baudrate` | 串口设备与波特率（256000，非标准值，用 `termios2+BOTHER` 设置） |
| `motors[].addr` | 从站地址（1~3） |
| `motors[].microstep` | 细分（初始化时写入 `0x65`，默认 16） |
| `motors[].speed_rpm` | 下发速度，**电机轴** RPM（1~6000）。关节角速度 = `speed_rpm × 6 ÷ 减速比` |
| `motors[].accel` | 加减速**档位** 0~200（手册 4.4.3）：200 最猛，0 = 无加减速直接启停 |
| `joint.gear_ratio` | 三个主动臂减速比（电机角 = 关节角 × 减速比） |
| `joint.dir` | 每轴方向系数 `+1/-1`，用于匹配机械安装方向（某轴转反了就改这一轴） |
| `joint.min_pos` / `joint.max_pos` | 三个关节的**角度限幅**（度），每次下发前夹取 |

缺失/非法时的回落：`speed_rpm`→300 RPM、`accel`→档位 30、`gear_ratio`→1.0、`dir`→+1、限幅→±180°，
全部会打印醒目告警；启动日志会回显 `减速比 / 关节方向 / 关节限幅 / 下发参数` 便于核对。

## 控制循环（`ArmDriveTask::run()`）

1. 周期统计（每 1000 拍打印一次实际频率）；
2. 读共享内存 `ArmCommand`（三个关节角 + `mode`）；
3. 发布 `ArmStatus`：`seq` 每拍递增（工具靠它判断底层是否在运行）、`state` bit0、`online[]`；
4. 首次下发前 `ensureDrivesReady()`：每 100 ms 轮转探测一台 → 三台在线后逐台 `initMotor`（`0xFB` 清状态 + `0x69` 协议=Modbus + `0x65` 细分）→ `enableMotors`（`0xFA`）；
5. 已判定掉线的电机：每 1 s 轮转重探一台，通了自动重新纳入下发；
6. `mode != 1` 视为空闲，不下发；
7. 每拍只给**一台**电机下发，`1→2→3` 轮转（`thread_t=4ms` → 每台约 83 Hz）；
8. 每个轴下发前：**NaN/Inf 检查** → **关节角限幅** → **× `joint.dir`** → **× `joint.gear_ratio`** → 计数（负角度按 int32 补码）→ `0xF2` 绝对位置（速度/加减速取自配置）；
9. 在线判定：某台**连续 10 次无应答**（约 150 ms）时单独探一次——有应答就继续、无应答判掉线（`online[i]=0`，跳过该轴）；CRC 错 / 从站异常码算"有应答"，不计入掉线。

日志只在状态变化时打印（限幅被触发/解除、失败原因变化、掉线/恢复），流式下发不会刷屏。

## 共享内存契约（上位机 ↔ 底层）

POSIX 共享内存 `/delta_arm` + System V 信号量（key `0x5A11`），共 **128 字节**：

```c
struct ArmCommand {          // 上位机 → 底层
    uint64_t seq;            // 写入序号（每次写 +1）
    uint8_t  mode;           // 0=空闲  1=按 theta 下发
    uint8_t  reserved[3];    // 对齐填充
    uint32_t magic;          // 布局魔数 0x444C5431（'DLT1'）
    double   theta[3];       // 三个【关节角】（度），相对零点位形
};

struct ArmStatus {           // 底层 → 上位机
    uint64_t seq;            // = 控制循环计数（每拍递增，判活用）
    uint8_t  state;          // bit0=正在按命令下发；bit1=实测有效（暂未实现）
    uint8_t  online[3];      // 各电机在线标志（真实值）
    double   theta[3];       // 电机角（度）——未回读，暂为 0
    double   motor_pos[3];   // 电机计数    ——未回读，暂为 0
    double   joint_theta[3]; // 关节角（度）——未回读，暂为 0
};
```

- **每次启动整块清零**（`mode=0`）：所以顺序是「先启动底层，再让上位机发命令」，残留命令不会被执行；
- `magic` 不符（旧布局/别的程序留下的同尺寸内存）也会被清零；
- 目前只有 `seq / state / online` 是有效信息，角度和计数要等回读实现。

## 工具脚本（`tools/`）

| 脚本 | 用途 |
| --- | --- |
| `probe_bus.py` | 总线探测/排障：地址扫描、成功率、只听不发、`--raw` 注入任意帧（内置模式只发 `04H` 读，不会让电机动作） |
| `read_angles.py` | **只读**打印三台角度：自动选数据源（底层在跑读共享内存，没跑则直连串口）、CRC 校验 + 重试、应用 `joint.dir`；`--capture/--ref` 显示相对角度、`--json` 纯 JSON、`--dev` 指定串口 |
| `set_angle.py` | 向共享内存写三个关节角（`--idle` 置空闲、`--unit joint\|motor`、`--hint-jump` 只提示）；底层没在运行时拒绝写入 |
| `set_zero.py` | **以当前位置为零点**（驱动器 `0xF8`）：直接运行即可；`--dry-run` 只读、`--axis` 选轴、`--ask` 才询问；底层在跑时拒绝 |
| `testpy.py` | 串口直连调试（读版本/位置/转速/状态），参数写死，需要 `pyserial` |
| `docs/` | PDxxS1 用户手册、Modbus 协议表、机构简图 |

```bash
cd delta_bottom
python3 tools/probe_bus.py --baud 256000 --addr 1-3 --trials 5   # 三台是否都在线
python3 tools/read_angles.py                                     # 当前角度
python3 tools/set_angle.py 5 0 0                                 # 下发关节角（写入即生效）
python3 tools/set_angle.py --idle                                # 置空闲
python3 tools/set_zero.py                                        # 以当前位置为零点
```

## Modbus 寄存器与总线时序

功能码：`04H` 读输入寄存器、`06H` 写单寄存器、`10H` 写多寄存器（自定义协议的功能码放在寄存器地址里）。

| 寄存器 | 作用 | 寄存器 | 作用 |
| --- | --- | --- | --- |
| `0x20` | 读软硬件版本 | `0x65` / `0x69` | 设置细分 / 设置协议 |
| `0x23 / 0x24` | 相电流 / 总线电压 | `0x67` | 设置串口波特率 |
| `0x29 / 0x2A / 0x2B` | 转速 / **位置** / 位置误差 | `0x6A / 0x6B` | 堵转保护 / 堵转电流 |
| `0x2C / 0x2D / 0x2E` | 运行状态 / 堵转标志 / 堵转电流 | `0xF2 / 0xF3` | 绝对 / 相对位置控制 |
| `0x2F / 0x30` | **使能状态** / 到位标志 | `0xF8 / 0xF9` | 位置清零 / 解除堵转保护 |
| `0x32` | 读驱动参数 | `0xFA / 0xFB / 0xFC` | 使能 / 清除状态 / 立即停止（刹车） |
| `0x91 / 0x92 / 0x96` | 限位回零 / 触发回零 / 回零状态 | | |

- `0x2C` 运行状态：0 停止、1 任务完成、2 运行中、**3 过载、4 堵转、5 欠压**。
- 手册 5.1.3 时序：**两条完整指令间隔 ≥ 2 ms**，整帧必须连续发送。
- 实测一次事务约 **2.5~3.2 ms**（256000 bps 下线路只占约 0.7 ms，其余是驱动器响应延迟），所以一个周期只放一次事务。
- `0xF2` 的注意点（手册 4.4.3）：绝对位置模式下，到达目标后**重复发同一条指令不会继续动作**，除非改目标或清零位置。

## 零点标定

驱动器返回的是它自己的绝对计数，和「θ=0 的位形」无关；**不标定就发绝对角度会大幅乱动**。

1. 人工把机械臂摆到参考位形（例如三个上臂水平）；
2. 停掉 `delta_bottom`，执行 `python3 tools/set_zero.py`（以当前位置为零点）；
3. `python3 tools/read_angles.py` 确认三台都变成 0 附近。

等价的原始帧（`06H` 写 `0x00F8` = 1，可用 `probe_bus.py --raw` 发）：

```
01 06 00 F8 00 01 C9 FB     02 06 00 F8 00 01 C9 C8     03 06 00 F8 00 01 C8 19
```

> 这类驱动器**不是掉电记忆**的：重新上电后计数会回到 0，需要重新标定，或用 `0x91/0x92` 做回零。

## 已知问题（实测）

1. **驱动器偶发坏帧**：1 号（偶尔 2/3 号）返回的帧**最后一字节**会被破坏（实测 `CRC DA B5` 收到 `DA BD`，翻 1 bit），偶尔还会多吐异常帧（`01 84 04` / `01 84 01`）。软件侧只做到"CRC 校验 + 重试 + 只把超时算掉线"，物理层需查**共地、120Ω 终端电阻、A/B 双绞与屏蔽**。
2. **停机后角度会缓慢漂移**（实测 2~3° 电机角）：停机后没有刚性保持，需要保持时得显式刹车/使能。
3. **`argv[1]` 双重语义**：线程配置与硬件配置共用一个路径参数。
4. **没有实测回读**：`ArmStatus` 的角度字段恒为 0，所以 `read_angles.py` 在底层运行时只能显示 0；也做不到"实测越界检查"。
5. **没有硬件限位**：只做了指令级软件限幅；驱动器支持限位开关回零（`0x91/0x92/0x96/0x99`）但未使用。
6. **掉线后不会自动恢复运行**：只有在线标志会自动恢复，运动逻辑本身没有"故障恢复"策略（不会补 `0xF9/0xFB` 再使能）。

## 待办

1. **位置回读**：轮转读 `0x2A`，填 `ArmStatus` 的 `theta/motor_pos/joint_theta` 并置 `state` bit1；
2. 基于回读做**实测越界检查**（超限停发/报警）；
3. **驱动器状态监控**：`0x2C`（过载/堵转/欠压）、`0x2D`、`0x2F`，异常时停机并给出原因；
4. 故障恢复：连续异常码时补 `0xF9`（解除堵转）+ `0xFB`（清状态）+ `0xFA`（使能）；
5. 限位开关回零（`0x91/0x92`）替代人工置零；
6. 串口单实例锁（`flock`），避免两个主站同时占总线。
