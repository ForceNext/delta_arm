# delta_arm

Delta 并联机械臂项目。当前已实现下位机 `delta_bottom`：在 Linux 上通过串口（RS485 / USB 转串口）以 **Modbus RTU** 协议控制 **3 台正点原子 PDxxS1 闭环步进电机驱动器**，以固定周期（默认 **200 Hz / 5 ms**）从共享内存读取三个**关节角**，乘减速比换算成计数后下发电机绝对位置指令；**坐标正逆解已移到上位机**，本工程只负责角度→计数→Modbus 下发与实测回读。

## 当前状态

- **已实现（可运行）**：实时任务框架 `PeriodicTask`（`timerfd` 周期调度 + `SCHED_FIFO` 实时优先级 + CPU 亲和性绑定 + 线程生命周期管理）；串口驱动 `SerialPort`（含任意波特率与帧间静默等待）、Modbus 主站 `ModbusMaster`（含失败原因返回码）、电机控制封装 `MotorMapping`、delta 正逆解 `Kinematics`、上下位机共享内存 `ArmSharedMemory`，以及机械臂驱动任务 `ArmDriveTask`（在线监测/初始化 → 角度换算 → 下发/回读交错 → 发布实测关节角 → 掉线保护）。
- **控制链路已贯通（角度接口）**：`ArmDriveTask::run()` 在 5 ms 周期内完成「读 `ArmCommand.theta[3]`（三个关节角）→ ×减速比 → `DEG2CNT` 计数 → 下发 `moveAbsolute`」，并**在同一轮里回读电机实测位置 → ÷减速比 → 实测关节角 → 发布到 `ArmStatus.joint_theta[3]`**，上位机可直接用它做正解/显示。
- **单拍事务预算可控**：实测一次 Modbus 事务约 **2.6~3.2 ms**（其中线路传输只占约 0.7 ms，其余是驱动器响应延迟），每拍 3 台的旧方案需 5~8 ms、必然掉频（实测 148~162 Hz）；**一拍只做 1 次事务**（约 2.9~3.2 ms）才能稳在 5 ms 周期内。
- **失败原因可辨**：`ModbusMaster` 对外返回 `0=成功 / -1 超时 / -2 地址或功能码错 / -3 CRC 错 / >0 从站异常码`；`ArmDriveTask` 会打印「超时 / CRC 校验错 / 从站异常码 0x04 从站设备故障」等原因，且**只有"连续无应答（超时）"才判定掉线**，CRC/地址错与从站异常只统计不误判。
- **在线监测与初始化**：启动时持续探测 3 台电机，全部上线后逐台初始化（清除状态 `0xFB` → 设置协议 `0x69`=modbus → 设置细分 `0x65`，细分数值读自配置），初始化完成才使能并进入运动控制；运行中连续无应答（默认 20 次）判定掉线，对在线电机执行刹车保持（`0xFC`）**并随即清除刹车/故障状态（`0xFB`，手册 5.4.13 要求，避免驱动发热）**并停机，周期重探等待恢复。
- **未实现**：电机**零点偏移与转向标定**（当前 `θ=0` 直接对应上臂水平、电机角 = 关节角 × 减速比，尚未纳入各臂零点偏置/反向；运行前需人工把三台位置计数器清零，见下文「运行前自检与零点标定」）。

## 关键设计

- **控制周期**：默认 **5 ms（200 Hz）**，由 `timerfd`（`CLOCK_MONOTONIC`）精确唤醒，避免周期漂移
- **下发/回读策略**：**6 拍一轮交错**——`发M1 → 读M1 → 发M2 → 读M2 → 发M3 → 读M3`，每拍只做 1 次 Modbus 事务（下发约 3.2 ms / 回读约 2.9 ms），一轮 30 ms，**每台下发与实测回读各约 33.3 Hz**
- **实测关节角发布**：三台都读到实测位置后，`ArmStatus` 里的 `motor_pos/theta/joint_theta` 都改为**实测值**（计数、电机角、关节角）；实测值就绪时 `state` 的 bit1 置 1。
  **空闲时（`mode=0`）也继续回读**（只是不下发），因此上位机随时都能看到当前实测值，也才有"参考基准"可用
- **失败日志限频**：同一台电机最多每 **1 s** 打印一条失败，并附该段时间内的次数；从站异常码额外给一次提示。原因：驱动器在运动/忙碌时会**拒绝读位置命令**（回读返回从站异常码 0x04），实测一次运行可产生上百条，逐条打印会刷满屏幕
- **共享内存每次启动清零**：底层 `connect()` 时无论新建还是附加，都会把 `ArmSharedData` 整块清零（`mode=0`、角度归零）并打印清零原因。这样上一次运行遗留的 `mode=1` + 旧角度不会被当成有效命令。
  实测：先植入一条遗留 `mode=1` 命令再启动底层，日志打印"上次遗留 mode=1 …已清零"，且 1000 拍内 **0 次事务**（完全没下发）。
  旧版本/外来程序留下的同尺寸数据（布局魔数 `ARM_SHM_MAGIC` 不同）同样会被清掉；尺寸不符时 `createNew(allowOverwrite=true)` 会自动 `ftruncate` 重建
- **事务预算**：单次事务实测平均 2.85 ms、最大 3.18 ms（0x04 只读事务，256000 bps）；`0xF2` 下发帧约多 0.3 ms，合计约 3.2 ms < 5 ms。**一次事务就要 2.9~3.2 ms，因此一个周期内不可能同时做"下发 + 回读"同一台**（合计约 6.1 ms），必须拆到不同拍
- **角度链路（双向）**：下发时 `theta[3]` 的含义由开关 `APPLY_GEAR_RATIO_ON_SEND`（`user/task/src/arm_drive_task.cpp`）决定
  - `false`（**当前**）：`theta` 是**电机角** → `计数 = θ × (51200/360)`
  - `true`：`theta` 是**关节角** → `计数 = θ × 减速比 × (51200/360)`（`jointAngleToMotorPos`）
  - 回读方向与开关无关：`实测关节角 = 计数 ÷ (51200/360) ÷ 减速比`（`motorPosToJointAngle`）
  - 坐标正逆解与末端轨迹规划都在上位机；`joint.jointlength` 底层已不使用
- **实时调度**：`SCHED_FIFO` + 可配置优先级（默认 75），线程绑定到指定 CPU；无 `root` / `CAP_SYS_NICE` 权限时**自动降级为普通调度**继续运行
- **通信协议**：Modbus RTU（实际使用 `04H` 读输入寄存器 / `06H` 写单寄存器 / `10H` 写多寄存器）
- **驱动器**：正点原子 PDxxS1 闭环步进驱动器，位置分辨率 `51200` = 一圈（360°）
- **波特率**：`256000`（非标准值，经 `termios2 + BOTHER` 设置）；驱动器支持 0~3000000，**上下位机必须一致**（命令码 `0x67`）
- **总线静默**：每次发帧前清残留；**上一次事务若异常收尾，则先等总线连续静默 ≥`idle_gap_ms`(默认 1 ms) 再发**，避免上一帧的尾巴被下一次事务误当应答（背靠背连发时"越错越多"的根源）。正常路径不做额外等待，不占周期预算
- **超时**：首字节 20 ms；帧内字节间隔 12 ms（驱动器在电机运动/忙碌时回帧可能被拆成两段，5 ms 会误判超时）
- **上下位机通信**：POSIX 共享内存（`shm_open` + `mmap`）+ System V 信号量互斥（key `0x5A11`）
- **在线探测**：读 `0x2A`（实时位置）判断电机在线；启动时持续探测（**每轮只探一台，1→2→3 轮转**，避免不存在的地址每次堵住 20 ms），全部上线才初始化并运动
- **掉线保护**：运行中连续**无应答**（默认 20 次）判定掉线，`stopMotor` 刹车保持 + `clearStatus` 清状态，避免末端掉落与驱动过热

## 架构与数据流

### 控制链路

```
上位机(独立进程，不在本工程)          本工程 delta_bottom                     电机 ×3
─────────────────────────           ────────────────────────                ───────
写 ArmCommand{seq, mode, theta[3]}   ArmDriveTask 200Hz 循环，6 拍一轮：    PDxxS1
 theta = 三个【关节角】(度)           ① 读命令（sem_p 加锁）
  ─────── 共享内存 /delta_arm ────▶ ② 角度换算：计数 = θ × (51200/360)（开关为 true 时才再乘减速比）
                                    ③ 偶数拍下发 moveAbsolute（本拍只发一台）──Modbus RTU──▶
                                    ④ 奇数拍回读 0x2A 实测位置 ───────────────Modbus RTU──◀
                                       计数 ÷(51200/360) ÷减速比 → 实测关节角
                                    ⑤ 三台实测角齐 → 状态改为实测值
  ◀─────── 发布 ArmStatus ────────  ⑥ 发布状态（motor_pos/theta/joint_theta 为实测；bit1=1）
```

> 一轮 6 拍：`发M1 → 读M1 → 发M2 → 读M2 → 发M3 → 读M3`（每拍 1 次事务，约 2.9~3.2 ms < 5 ms），每台下发与实测回读各约 33.3 Hz。
> `ArmDriveTask` 在 `init()` 中完成共享内存连接与配置读取；`run()` 先走「等待上电（轮转探测）→ 初始化（协议+细分）→ 使能」的启动流程，就绪后按上述 6 拍循环运行，并在运行中持续监测电机在线状态。

### 分层调用链

```
main.cpp（应用层：读 thread_config → 建 ArmDriveTask → 绑定 CPU/优先级 → start）
   │ ArmDriveTask::run()（在线监测/初始化 → 角度换算 → 下发 → 回读实测 → 发布状态）
   ▼
MotorMapping（drives/：在线探测/初始化/使能/运动控制 + 角度↔计数换算 + 减速比，组命令码）
   │ moveAbsoluteRc() 下发 ／ readMotorPosRc() 回读实测 ／ probeMotorRc() 探测
   │ jointAngleToMotorPos() ／ motorPosToJointAngle()  ← 含减速比的关节角↔计数
   │ gearRatio() ／ loadGearRatios()  ← 从 hardware_config.yaml 读减速比
   │ 以上 *Rc 版本都返回 0/-1/-2/-3/>0，可区分超时、CRC 错、从站异常码
   ▼
（坐标正逆解已完全移到上位机；底层不再包含任何 IK/FK 代码，`kinematics/` 目录已删除）
   ▼
ModbusMaster（drives/：组帧 + CRC-16 + 发送 + 两段超时收应答 + 校验 + 异常收尾静默）
   │ readExact() / write() / waitIdle() / flushInput()
   ▼
SerialPort（drives/：termios 串口，8N1 原始模式，半双工 tcdrain，支持任意波特率）
```

## 实时任务框架（`PeriodicTask`）

`common/Utilities/PeriodicTask` 是本工程的控制线程基础设施：

- **周期调度**：每个任务线程内部用 `timerfd` 按 `period` 精确唤醒（`func()` 中 `read(timerFd)` 阻塞到下一拍）；**不追赶**已经错过的节拍（`read` 返回的过期次数被丢弃）。
- **实时优先级**：`start()` 里通过 `pthread_attr_*` 设置 `SCHED_FIFO`；若 `pthread_create` 返回 `EPERM`（无 `root` / `CAP_SYS_NICE`），自动用默认属性重试，降级为普通调度。
- **CPU 亲和性**：`set_bind_cpu(id)` 把线程绑到指定核（`pthread_setaffinity_np`），并在绑定后回读校验。
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
| `SerialPort` | `drives/` | 硬件层：termios 串口 8N1、任意波特率（`termios2+BOTHER`）、`tcdrain` 半双工、`readExact` 精确收帧、`waitIdle` 帧间静默 | ✅ 已实现 |
| `ModbusMaster` | `drives/` | 协议层：组帧 + CRC + 同步事务（发→收→校验）+ 失败原因返回码 + 异常收尾静默 | ✅ 已实现 |
| `MotorMapping` | `drives/` | 在线探测/初始化/使能/运动控制 + 角度↔计数换算 + 减速比（含 `*Rc` 版本） | ✅ 已实现 |
| `arm_drive_task` | `user/task/` | 机械臂驱动任务（`PeriodicTask` 子类）：在线监测/初始化 → 关节角×减速比 → 6 拍交错下发/回读 → 实测关节角发布 + 掉线停机 | ✅ 已实现 |
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
    │   ├── hardware_config.yaml      # 串口 + 电机列表 + 关节几何参数
    │   └── thread_config.yaml        # 线程配置：周期 / 绑定 CPU / 实时优先级
    ├── common/                       # 公共库 libcommon.a
    │   ├── include/
    │   │   ├── Utilities/SharedMemory.h   # 共享内存（shm_open + sem_com 锁）
    │   │   ├── Utilities/PeriodicTask.h   # 实时任务框架
    │   │   ├── Utilities/sem_com.h        # System V 信号量互斥锁
    │   │   └── Utilities/*.h              # Timer/utilities/Utilities_print 等
    │   └── src/Utilities/
    ├── drives/                       # 驱动库 libdrives.a（串口 + Modbus + 电机映射）
    │   ├── include/{serial_port, modbus_crc, modbus_master, MotorMapping}.hpp
    │   └── src/{serial_port, modbus_master, MotorMapping}.cpp
    ├── user/                         # 应用层（入口 + 驱动任务）
    │   ├── main.cpp                  # 应用入口：读线程配置 → 建任务 → 保活退出
    │   └── task/                     # 机械臂驱动任务库 libtask.a（ArmDriveTask 控制循环）
    │       ├── CMakeLists.txt
    │       ├── include/arm_drive_task.hpp
    │       └── src/arm_drive_task.cpp
    ├── third_party/yaml-cpp/         # 配置解析（vendored）
    └── tools/
        ├── probe_bus.py              # 总线探测/排障（地址扫描、成功率、只听不发、--raw 注入任意帧）
        ├── read_angles.py            # 只读：打印三台绝对/相对角度（自动选数据源，避免抢总线）
        ├── set_angle.py              # Python 脚本：输入三个【角度】→ 写共享内存
        ├── testpy.py                 # Python 串口直连调试脚本（读版本/位置/转速/状态，需 pyserial）
        └── docs/                     # 驱动协议与手册
            ├── 正点原子步进电机驱动器modbus-rtu控制协议V1.2(2).xlsx
            ├── PDxxS1步进电机闭环驱动器用户手册_V1.0.pdf
            └── delta机械臂数学模型.png
```

## 通信协议

驱动器把 Modbus 的「寄存器起始地址」当作**命令码**使用（高字节补 0），读写均走标准 Modbus RTU 帧：

| 命令码 | 说明 | 本工程是否使用 |
| --- | --- | --- |
| `0x20` | 读软硬件版本 | 调试用 |
| `0x29` | 读实时转速（int16，单位 RPM） | 调试用 |
| `0x2A` | 读实时位置（int32，`51200` = 一圈） | ✅ 在线探测 |
| `0x2C` | 读运行状态 | 调试用 |
| `0x2F` | 读使能状态 | 调试用 |
| `0x30` | 读到位标志 | 调试用 |
| `0x60` | 设置从机地址（0~255，0=广播，出厂默认 1） | 调试/配置用 |
| `0x62` | 设置工作模式（0=通信位置 … 7=回零） | 封装已提供 |
| `0x65` | 设置细分（1~256） | ✅ 初始化 |
| `0x67` | 设置串口波特率（0~3000000） | 调试/配置用 |
| `0x69` | 设置协议（0=自定义 / 1=modbus / 2=canopen） | ✅ 初始化 |
| `0xF2` | 闭环绝对位置模式控制 | ✅ 运动下发 |
| `0xF3` | 闭环相对位置模式控制 | 封装已提供 |
| `0xF8` | 将当前位置角度清零（写 `0x0001`） | 标定用（需手动发送） |
| `0xFA` | 使能控制（0=使能 / 1=失能） | ✅ 初始化后使能 |
| `0xFB` | 清除状态（堵转/刹车/失能） | ✅ 初始化、急停后 |
| `0xFC` | 立即停止（刹车） | ✅ 掉线保护 |

闭环绝对位置（`0xF2`）数据区 8 字节 = 4 个寄存器：

| 字节 | 含义 |
| --- | --- |
| 1 | 电机旋转方向（0=正转 / 1=反转） |
| 2 | 加减速度（0~200，r/s²，0=直接启动） |
| 3~4 | 速度（0~6000 RPM，uint16） |
| 5~8 | 绝对位置（uint32，`51200` = 一圈） |

位置换算：`一圈 360° = 51200`，故 `90° = 12800`、`一圈/s = 51200 计数/s`。

## 运动参数与控制封装

`drives/MotorMapping` 封装了角度 ↔ 计数换算与运动控制（均为静态方法，`motorId` 直接对应从站地址 1~3）：

| 方法 | 功能 | 命令码 |
| --- | --- | --- |
| `loadMotorAddrs()` | 从 yaml 读候选电机地址与细分 | — |
| `loadGearRatios()` / `gearRatio(i)` | 从 yaml 读三个主动臂**减速比**（电机角 = 关节角 × 减速比） | — |
| `probeMotor()` / `probeMotorRc()` | 读 `0x2A` 探测单台在线（`Rc` 版返回失败原因） | `0x2A` |
| `readMotorPosRc(master, id, pos)` | 读实测位置（int32 计数）到 `pos`，返回失败原因 | `0x2A` |
| `allOnline()` | 配置里的所有电机是否都在线 | — |
| `initMotor()`（`clearStatus` + `setProtocol` + `setSubdivision`） | 初始化：清除状态 + 设置协议(modbus) + 设置细分(读自配置) | `0xFB` + `0x69` + `0x65` |
| `setMotorEnable()` / `enableMotors()` | 单台 / 全部使能（失能） | `0xFA` |
| `SetMotorMode()` | 设置工作模式 | `0x62` |
| `moveAbsolute()` / `moveAbsoluteRc()` | 绝对位置控制（`Rc` 版返回失败原因） | `0xF2` |
| `moveRelative()` | 相对位置控制 | `0xF3` |
| `stopMotor()` / `clearStatus()` | 立即停止（刹车）/ 清除状态 | `0xFC` / `0xFB` |
| `jointAngleToMotorPos(joint_angle, ratio)` | **下发方向**：关节角（度）× 减速比 × (51200/360) → 计数 | — |
| `motorPosToJointAngle(pos, ratio)` | **回读方向**：计数 ÷ (51200/360) ÷ 减速比 → 关节角（度） | — |
| `angleToMotorPos()` / `motorPosToAngle()` | 电机角（度）↔ 电机计数换算（**不含**减速比） | — |
| `isOnline()` / `setOnline()` / `errs()` | 在线标志读写 / 累计失败计数 | — |

> 运动参数（加减速、速度）通过 `moveAbsolute()` / `moveRelative()` 的默认参数传入（`accel=200`、`speed=1000 RPM`）；`ArmDriveTask` 里对应常量 `MOVE_ACCEL` / `MOVE_SPEED`。
>
> **注意**：`stopMotor()` 刹停后需及时调用 `clearStatus()` 清除状态，否则电机可能严重发烫（见手册 5.4.13）——`ArmDriveTask` 的急停分支已自动执行这一步（可用 `CLEAR_STATUS_AFTER_BRAKE` 常量关闭）。

### 事务失败原因返回码

`ModbusMaster` 对外暴露 `ModbusRc`，并由 `modbusRcName()` 转成可读文本；上层据此区分失败原因：

| 返回码 | 含义 | 是否计入"掉线"判定 |
| --- | --- | --- |
| `0` | 成功 | — |
| `-1` | 超时 / 应答长度不足 | ✅ 计入（连续 20 次判掉线） |
| `-2` | 从站地址或功能码不匹配 | ❌ 只统计并打印 |
| `-3` | CRC 校验错 | ❌ 只统计并打印 |
| `> 0` | 从站异常码（`0x01` 非法功能 / `0x02` 非法数据地址 / `0x03` 非法数据值 / `0x04` 从站设备故障…） | ❌ 只统计并打印 |

| 时间参数（`ModbusMaster` 成员） | 取值 | 说明 |
| --- | --- | --- |
| `resp_timeout_ms` | `20` | 等应答首字节 |
| `byte_timeout_ms` | `12` | 帧内字节间隔（驱动器忙碌时回帧可能分两段） |
| `idle_gap_ms` | `1` | 异常收尾后发帧前要求的总线静默时间（等待上限取其 4 倍或 20 ms） |

## 上下位机通信（共享内存）

上下位机**共用同一份头文件**（[common/include/Utilities/SharedMemory.h](delta_bottom/common/include/Utilities/SharedMemory.h)），保证内存布局一致：

```cpp
#define ARM_SHARED_MEMORY_NAME "/delta_arm"   // POSIX 共享内存名（须以 / 开头）
#define ARM_SEM_KEY            0x5A11         // System V 信号量 key（上下位机必须一致）

struct ArmCommand {          // 上位机 → 底层（坐标正逆解已移到上位机）
    uint64_t seq = 0;        // 写入序号（每次写 +1）
    uint8_t  mode = 0;       // 0=空闲 1=角度模式
    uint8_t  reserved[3];    // 对齐填充
    uint32_t magic;          // 布局魔数 ARM_SHM_MAGIC（不符则底层整块清零重建）
    double   theta[3];       // 三个【关节角】（度）；θ=0 为上臂水平，θ>0 上臂下压
};

struct ArmStatus {           // 底层 → 上位机
    uint64_t seq = 0;        // 写入序号（= 控制循环计数）
    uint8_t  state = 0;      // bit0=1 正在按命令运行；bit1=1 本次数据来自实测回读
    uint8_t  online[3];      // 各电机在线标志
    double   theta[3];       // 三个电机角（度，已乘减速比）：实测优先，未读到时为命令值
    double   motor_pos[3];   // 三个电机位置（计数）：实测优先，未读到时为命令值
    double   joint_theta[3]; // 三个【实测关节角】（度）= 计数 ÷(51200/360) ÷减速比
};

struct ArmSharedData {       // 共享内存整体布局（锁不在块内）
    ArmCommand command;
    ArmStatus  status;
};                           // 共 128 字节（C++ 有 static_assert，Python 侧有 sizeof/offset 自检）
```

- **共享内存**：POSIX `shm_open` + `mmap`，名字 `/delta_arm`。
- **互斥锁**：独立的 System V 信号量（[common/include/Utilities/sem_com.h](delta_bottom/common/include/Utilities/sem_com.h)），key = `0x5A11`，不在共享内存块内部。
- **读写**：每次读/写 `command` 或 `status` 前 `sem_p()` 加锁、之后 `sem_v()` 解锁。
- **首次创建**：信号量用 `IPC_CREAT | IPC_EXCL` 判断是否首次创建——首次创建者初始化为 1，其余进程只打开不重置；`SEM_UNDO` 保证进程异常退出时自动释放持有的锁。
- **启动清零（重要）**：底层**每次启动**都会把 `ArmSharedData` 整块清零。因此在底层启动**之前**由上位机写入的命令会被丢弃——请**先启动底层，再让上位机开始下发**。

`ArmSharedMemory` 类封装了上述逻辑：

```cpp
ArmSharedMemory shm;           // 构造时打开信号量锁（key = ARM_SEM_KEY）
shm.connect();                 // 首次创建 / 其余附加共享内存

ArmCommand cmd;
shm.readCommand(cmd);          // 底层读命令（加锁 → 读 → 解锁）
ArmStatus st;
shm.writeStatus(st);           // 底层写状态（加锁 → 写 → 解锁）
```

### 单点角度脚本（`tools/set_angle.py`）

真正的上位机就绪前，用 Python 脚本向共享内存发布三个**关节角**：

```bash
cd delta_bottom
python3 tools/set_angle.py 0 0 0        # 三个关节角（度），按电机 1/2/3 顺序
python3 tools/set_angle.py -10 5 20     # 也可给负角度（需已做过位置计数器零点标定）
python3 tools/set_angle.py              # 不带参数则交互式输入
```

- 先启动底层 `./build/delta_bottom`，再运行该脚本。
- 脚本用 `ctypes` 定义与 C++ 一致的 `ArmCommand/ArmStatus` 布局（共 128 字节，并断言各字段偏移），用 System V 信号量（key `0x5A11`）加锁，并写入布局魔数。
- **角度含义**：当前 `APPLY_GEAR_RATIO_ON_SEND=false`，`theta` 直接作为**电机角**下发（`计数 = θ × 51200/360`）；
  置为 `true` 则 `theta` 是关节角（底层先 ×减速比）。改开关后请同步告知上位机侧（脚本用 `--unit motor|joint` 声明）。
- **护栏（`set_angle.py`，可用 `--force` 跳过）——只拦"拿垃圾当基准"，不做跳变限幅**：
  ① 底层没在运行（`status.seq` 不变化）→ 拒绝（共享内存里可能是上次的陈旧状态，不能当基准）；
  ② 底层还没回传实测值（`state` bit1=0）→ 拒绝（否则等于以 0 为基准）；
  ③ 跳变超过 `--hint-jump`（默认 20°，单位同 `--unit`）→ **只打印提示，照常下发**（不限幅）。
- 随后回显底层 `state/online/seq`、实测电机角与**实测关节角**（实测关节角始终按 ÷减速比 计算）。
- `--idle`：只写 `mode=0`（空闲），用于脚本异常退出后清掉残留的 `mode=1`。

### 读角度脚本（`tools/read_angles.py`）

**只读**工具，打印三台的计数 / 电机角 / 关节角。注意：驱动器**没有**"读相对位置"的寄存器
（`0xF3 相对位置模式控制` 是一条**写**命令，"每次相对当前位置偏移"），可读的绝对量只有
`0x2A 读电机实时位置`；因此"相对角度"由参考点算出来：`相对计数 = 当前计数 − 参考计数`。

```bash
cd delta_bottom
python3 tools/read_angles.py                       # 绝对角度（+已保存参考点的相对角度）
python3 tools/read_angles.py --capture              # 把当前计数记为参考点（存 tools/.angle_ref.json）
python3 tools/read_angles.py --ref 33717,17682,12272  # 临时指定参考点（不保存）
python3 tools/read_angles.py --clear                # 清除参考点
python3 tools/read_angles.py --json                 # 机器可读
```

- **数据源自动选择**（关键：避免两个主站抢同一根 RS485）：底层在跑（`status.seq` 在变化）→ 读共享内存；
  底层没跑 → 直连串口读 `0x2A`。因此**底层运行时也能安全查看角度**。
- 典型用法（测量"一条命令实际走了多少"）：
  ```bash
  python3 tools/read_angles.py --capture   # 命令前记基准
  python3 tools/set_angle.py 60 45 30      # 下发
  python3 tools/read_angles.py             # 自动显示相对变化
  ```
- 参考文件 `tools/.angle_ref.json` 是本机运行状态，已加入 `.gitignore`。
- 离线的电机其数值会标注"不可信"（底层运行时取自共享内存，可能是掉线前的旧值）。

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
./build/delta_bottom 路径/到/配置.yaml      # 指定配置文件
```

Ctrl-C（或 `SIGTERM`）触发优雅退出：停止任务线程并执行 `cleanup()`。

> ⚠ **`argv[1]` 目前是"双重语义"**：`main()` 用它读**线程配置**，而 `SerialPort` / `ArmDriveTask` 也用它读**硬件配置**。因此传入自定义路径时会同时覆盖两份配置的路径——若该文件里没有 `serial:` / `joint:` 段，程序会**静默回退到默认值**（`/dev/ttyUSB0`、`921600`、默认关节参数），可能连错串口。建议：不加参数直接运行，或让该文件同时包含 `interface_task` 与 `serial`/`motors`/`joint` 三段。

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

> 每台的**下发频率 = 回读频率 = `1 / (thread_t × 6)`**（6 拍一轮：发1 读1 发2 读2 发3 读3），5 ms 周期下约 **33.3 Hz**。
> 之所以要拆成 6 拍：一次事务约 2.9~3.2 ms，一个周期内只装得下 1 次；「下发 + 回读」同一台要 6.1 ms，所以必须分拍。
> 若想让三台在同一拍都收到指令，需要把 `thread_t` 放宽到 `0.010` 以上（三台约 10 ms），代价是下发频率反而更低。

`delta_bottom/configs/hardware_config.yaml`（串口 + 电机列表 + 减速比，由 `SerialPort` / `MotorMapping` 读取）：

```yaml
serial:
  port: "/dev/ttyACM0"     # 串口设备
  baudrate: 256000         # 波特率（非标准值经 termios2+BOTHER 设置；必须与驱动器一致）

motors:                    # 电机（从站）列表
  - addr: 1                # 从站地址（0=广播，1~255 从站；本程序内部只支持 1~3）
    name: "motor_1"
    speed_rpm: 1000        # 预留字段
    accel: 200             # 预留字段
    microstep: 16          # 细分（microstep），初始化时经 0x65 下发
  - addr: 2
    name: "motor_2"
    speed_rpm: 1000
    accel: 200
    microstep: 16
  - addr: 3
    name: "motor_3"
    speed_rpm: 1000
    accel: 200
    microstep: 16

joint:                     # delta 几何参数
  jointlength:             # mm
    - [115, 150, 292, 40]  # d1(基座半径), L1(上臂长), L2(前臂长), d2(动平台半径)
  gear_ratio:
    - [3.0, 3.0, 3.0]      # 三个主动臂传动比（电机角 = 关节角 × 传动比）
```

- `serial.port` / `serial.baudrate`：串口与波特率。
- `motors[].addr`：驱动器从站地址；程序启动时持续探测，**等所有配置的电机都上线**才初始化并运动，任一掉线则停机。
  - ⚠ **配置里的地址必须与驱动器实际地址完全一致**：只要有一个地址没有设备，程序会永远停在「等待电机上电」，一台也不会使能。改动地址可用驱动器上位机/面板，或用命令码 `0x60`。
- `motors[].microstep`：细分值，由 `loadMotorAddrs` 读取，初始化时经 `0x65` 下发（默认 16）。
- `joint.jointlength`：基座半径 `d1`、上臂长 `L1`、前臂长 `L2`、动平台半径 `d2`（mm）。**底层已不再使用**（逆解在上位机），保留供上位机几何建模参考。
- `joint.gear_ratio`：三个主动臂**减速比**，由 `MotorMapping::loadGearRatios` 读取；底层用它把上位机给的关节角换算成电机角（`电机角 = 关节角 × 减速比`）。⚠ 读不到时会按 1.0 处理并醒目告警——差一个倍数很危险。
- `motors[].name` / `speed_rpm` / `accel` 为预留字段，暂未在控制循环中使用（加减速/速度用 `arm_drive_task.cpp` 里的 `MOVE_ACCEL` / `MOVE_SPEED`）。

## 运行前自检与零点标定

### 1. 从站在线自检（推荐每次都做）

`tools/probe_bus.py` 是总线探测工具：**内置的地址扫描 / 成功率统计 / 只听不发模式只发送 `04H` 读命令**，不会让电机动作；`--raw` 则按你给定的字节原样发送（可用于发零点的 `0xF8` 等写命令，务必自行确认帧内容）。可用来确认串口、波特率、从站地址是否正确：

```bash
cd delta_bottom
python3 tools/probe_bus.py --baud 256000 --addr 1-8 --trials 5     # 地址扫描 + 成功率统计
python3 tools/probe_bus.py --baud 256000 --addr 247 --listen 3     # 只听不发：看总线是否被某台从站吐帧霸占
python3 tools/probe_bus.py --baud 256000 --raw 0304002A000251E1    # 注入一帧，打印原始应答（诊断用）
python3 tools/probe_bus.py --baud 115200,256000,921600 --addr 1    # 多波特率对比
```

它会打印每台的原始应答、首字节延迟，并把结果分类为 `OK / BAD_CRC / ADDR_MISMATCH / EXCEPTION(码) / TIMEOUT`——这正是排查「电机一直来回不在线」的第一手段。正常状态应看到三台都是 100%。

### 2. 位置计数器零点标定（首次运行或更换驱动器后必做）

程序下发的是**绝对位置**（`计数 = 关节角 × 减速比 × 51200/360`），而驱动器上电后的位置计数器是它自己的零点。
**换成角度接口后这一步更不能省**：上位机算出的关节角是相对"参考位形（θ=0 = 上臂水平）"的，若三台计数器零点与它不一致，
第一个角度指令就会让电机**高速大幅转动**（例如把关节角 0° 发给一台停在 53°（关节）的电机 → 反转约 160° 电机角），可能撞限位或损坏机构。

> 调试阶段若要避免"第一个绝对角度指令就是大幅跳变"，可以先用
> `python3 tools/read_angles.py --capture` 记下当前姿态作基准，再发小幅角度；
> 但**正式接入上位机前，务必先按下面流程把三台计数器清零**，否则上位机算出的绝对角度没有物理意义。

标定流程：

1. 人工把机械臂摆到参考位形（例如三个上臂水平）；
2. 用命令码 `0xF8`（写单寄存器 `0x0001`）把三台位置计数器清零：

    ```
    地址1: 01 06 00 F8 00 01 C9 FB
    地址2: 02 06 00 F8 00 01 C9 C8
    地址3: 03 06 00 F8 00 01 C8 19
    ```

    也可用 `probe_bus.py --raw <帧>` 发送（注意：它是写命令，会让位置计数归零）；
3. 之后启动 `delta_bottom`，三台就在同一参考下工作。

> 当前程序**不会**自动清零，也尚未实现各臂零点偏置/转向标定，所以这一步必须人工做。

## 硬件与依赖

- Linux（使用 `termios`、`select`、`timerfd`、`pthread` 实时调度）
- C++17
- CMake ≥ 3.16
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)（vendored 于 `third_party/yaml-cpp`）
- [Eigen3](https://eigen.tuxfamily.org/)（`common/include/cppTypes.h` 等遗留类型定义依赖，当前控制循环未直接使用）
- Python 3（可选，`tools/` 脚本；`testpy.py` 需要 `pip install pyserial`）

## 调试与排障

### 运行时日志

| 输出 | 含义 |
| --- | --- |
| `[CPU] Set thread affinity to CPU N success/failed` | CPU 亲和性设置结果 |
| `[PeriodicTask] ... 降级为普通调度重试` | 无 `SCHED_FIFO` 权限，已降级 |
| `[DriveTask] 等待电机上电：在线 XYZ` | 启动探测的在线状态变化（每轮轮转探一台） |
| `[DriveTask] 全部在线并初始化完成，已使能，开始运行` | 进入运动控制 |
| `[DriveTask] 电机 N 下发失败：超时/应答长度不足（-1）` | **只有这一类**累计后会判掉线 |
| `[DriveTask] 电机 N 探测异常：CRC 校验错（-3）` | 帧错/异常，只统计不判离线（**每台每 1 s 最多一条**，附近段时间次数） |
| `[DriveTask] 电机 N 连续无应答 20 次，判定掉线，停机！` | 急停：刹车保持 + 清状态后等待人工重启 |
| `[DriveTask] 电机 N 已恢复通信，请重启程序恢复运行` | 停机后重探成功（不会自动恢复运行） |
| `[DriveTask] 1000 loops: x.xxxxxx s -> yyy.yy Hz` | 实际循环频率，应稳定在设定值附近 |
| `[Modbus] 1000 loops: N 次事务（下发 A / 回读 B）\| 平均 ms/次 \| 最大 ms/次 \| 每拍合计 ms \| 非超时错误 N` | 事务耗时与帧错统计；「每拍合计」应明显小于周期（每拍只做 1 次事务） |
| `[DriveTask] 电机 N 回读失败：超时/应答长度不足（-1）` | 实测回读失败；原因与下发共用同一套返回码，仅超时累计判掉线（限频打印） |
| `[提示] 从站异常码通常是驱动器忙/拒绝该命令…` | 首次遇到从站异常码时给一次解释：回读失败只影响实测值刷新，不影响下发、不计入掉线 |

### 典型故障与处理

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 一直停在「等待电机上电」 | 某个 `addr` 在总线上没有设备／地址不符／串口或波特率错 | 用 `probe_bus.py --addr 1-8 --trials 5` 扫描实际应答的地址，改 `hardware_config.yaml` 或改驱动器地址（`0x60`） |
| 电机来回上下线 | 总线被某台从站持续吐帧霸占（表现为收到成片 `03 84 01/04 …` 异常帧），或帧被干扰损坏 | `probe_bus.py --addr 247 --listen 3` 判断是否被霸占；对那台驱动器断电重启，检查其供电与 RS485 接线 |
| 频率低于设定值 | 单拍事务量超过周期（旧版每拍 3 台需 5~8 ms） | 现为每拍 1 次事务、默认周期 5 ms；若仍不够，把 `thread_t` 放宽到 `0.008~0.010` |
| 频繁 `CRC 校验错` | 物理层问题：终端电阻、共地、双绞、分支线、驱动器供电 | 两端各一个 120 Ω 终端电阻、A/B 用双绞、与三台驱动器共地、缩短分支；必要时降低波特率（`0x67`，两端一致） |
| 打印 `设置默认参数 -- /dev/ttyUSB0, 921600` | `argv[1]` 被当成硬件配置路径（见「运行」一节的说明） | 不带参数运行，或让该配置同时包含线程段与硬件段 |
| 共享内存尺寸不符 / 连不上 | 残留旧 `/dev/shm/delta_arm` | 底层每次启动都会清零，尺寸不符时也会自动 `ftruncate` 重建，通常无需手工清理；要彻底重置：先停程序，再 `sudo rm -f /dev/shm/delta_arm`（信号量可留，需要时 `ipcrm -s <semid>`） |

其他手段：

- `tools/testpy.py`：绕过程序直接用串口读版本/位置/转速/状态/电压/电流/使能/到位，用于验证接线与通信（**注意其顶部常量写死为 `/dev/ttyACM0 @ 921600`、地址 1，需按实际修改，且需 `pyserial`**）。
- `PeriodicTaskManager::printStatus()` / `printStatusOfSlowTasks()`：打印各任务的最短/最长运行时长与周期，用于定位超时任务。

## 已知限制

1. **无零点偏移/转向标定**：`θ=0` 直接对应上臂水平、电机角 = 关节角 × 减速比，且 `moveAbsolute` 的 `direction` 固定为 0（正转）。
   负角度现在按 **int32 补码**编码下发（`(uint32_t)(int32_t)llround(...)`），而驱动器位置本身就是 int32，所以负角度可用；前提仍是「位置计数器零点标定」，否则绝对角度没有共同基准。各臂零点偏置/转向（反向）仍待实现。
2. **实测回读的频率与陈旧度**：受事务预算限制，回读与下发交错进行，每台约 **33.3 Hz**；若某次回读失败，该台继续沿用上一次实测值（`state` bit1 仍为 1），因此发布的实测关节角最大可能有 1~2 轮的陈旧度（约 30~60 ms）。实现上只在三台都读到之后才把整个状态切到实测值，避免出现"一台实测 + 两台命令"的混合位形。
3. **`argv[1]` 双重语义**：线程配置与硬件配置共用同一路径参数（见「运行」一节）。
4. **探测仍会占用一拍**：等待上电/停机重探已改为每轮一台，但若该地址无应答，仍会阻塞一个 20 ms 超时（每 20 拍或 200 拍发生一次），使频率短暂下探。
5. **对物理层较敏感**：256000 bps 下，终端电阻/共地/双绞/分支线不合格时会出现间歇性 CRC 错或从站异常码；软件侧只做到「不误判掉线、不连锁污染、打印真实原因」，无法修复被干扰的帧。
6. **`tools/testpy.py` 参数写死**（`/dev/ttyACM0 @ 921600`、地址 1），与本工程当前配置不一致，直连调试前需先改；它还需要 `pyserial`。
7. **底层已不含 IK/FK**：`Kinematics.{hpp,cpp}` 与 `kinematics/` 目录已随结构调整删除（坐标正逆解完全由上位机负责），`joint.jointlength` 底层也不再读取。如需那份三球交会的参考实现，可从 git 历史取回：`git show HEAD:delta_bottom/kinematics/src/Kinematics.cpp`。

## 后续规划（路线图）

以下为**尚未实现**的部分，供后续开发参考：

1. **零点偏移与转向标定**：把各臂零点偏置与转向（反向）作为配置项，在下发前并入角度换算（`θ_motor = dir × θ × 减速比 + offset`），替代当前"人工清零"的做法。
2. **到位判定与轨迹闭环**：已实现实测回读与实测关节角发布（6 拍交错，33.3 Hz；空闲时也在回读）。下一步可用 `0x30` 到位标志 / 位置误差 `0x2B` 做到位判定，或把实测关节角反馈给上位机做闭环。
3. **拆分配置路径**：为线程配置与硬件配置提供独立命令行参数，消除 `argv[1]` 双重语义。
4. **探测非阻塞化/异步化**：把等待上电与停机重探做成异步事务，彻底消除 20 ms 阻塞对周期的影响。
5. **共享内存无锁化（可选）**：用 seqlock 替代 System V 信号量，读者永不阻塞，更适合实时循环。
6. **补充测试**：为 `modbus_crc`（标准测试向量）、角度换算往返、共享内存布局补充测试（`sizeof(ArmSharedData)==128` 已有 `static_assert`）。
