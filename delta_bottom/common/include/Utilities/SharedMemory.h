/*! @file SharedMemory.h
 *  @brief delta_arm 上下位机共享内存
 *  底层（delta_bottom）与上位机共用同一块共享内存：
 *    上位机写入 ArmCommand，底层读取并下发。
 */
#ifndef PROJECT_SHARED_MEMORY_H
#define PROJECT_SHARED_MEMORY_H

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include "Utilities/sem_com.h"
#include "Utilities/Utilities_print.h"

// 共享内存名称（shm_open 的名字须以 '/' 开头）
#define ARM_SHARED_MEMORY_NAME "/delta_arm"

// System V 信号量 key（上下位机必须使用同一个 key，任意非零整数，避免与其他程序冲突）
#define ARM_SEM_KEY 0x5A11

// 共享内存布局版本（放在 ArmCommand 的魔数字段里）。
// 布局一变就改这个值：底层 connect() 时若发现魔数不符，会把整块内存清零，
// 避免"尺寸相同但字段含义变了"的旧数据被按新布局误读（例如旧 x/y/z 被当成关节角 → 电机大幅乱动）。
#define ARM_SHM_MAGIC 0x444C5431u   // 'D''L''T''1'

// 上位机 -> 底层：命令（三个【关节角】，单位度；坐标正逆解已移到上位机）
// 注意：这里传的是关节角，底层负责 电机角 = 关节角 × 减速比(hardware_config.yaml 的 gear_ratio)，
//       再 ×(51200/360) 转成计数下发。
struct ArmCommand
{
  uint64_t seq = 0;                        // 写入序号（每次写 +1）
  uint8_t mode = 0;                        // 0=空闲  1=角度模式（底层直接下发这三个关节角）
  uint8_t reserved[3] = {0, 0, 0};         // 对齐填充（勿使用）
  uint32_t magic = ARM_SHM_MAGIC;          // 布局版本魔数（见 ARM_SHM_MAGIC）
  double theta[3] = {0.0, 0.0, 0.0};       // 三个角度（度）。是【关节角】还是【电机角】由底层实现决定：
                                           //   当前 run() 是骨架、未实现下发；重写下发逻辑时需与
                                           //   tools/*.py 的 --unit motor|joint 保持一致
};

// 底层 -> 上位机：状态
struct ArmStatus
{
  uint64_t seq = 0;                          // 写入序号（= 控制循环计数）
  uint8_t state = 0;                         // bit0=1 正在按命令运行；bit1=1 本次数据来自实测回读
  uint8_t online[3] = {0, 0, 0};             // 各电机在线标志
  double theta[3] = {0, 0, 0};               // 三个电机角（度，已乘减速比）：实测优先，未读到时为命令值
  double motor_pos[3] = {0, 0, 0};           // 三个电机位置（计数）：实测优先，未读到时为命令值
  double joint_theta[3] = {0, 0, 0};         // 三个【实测关节角】（度）= 计数 ÷(51200/360) ÷减速比
};                                           // 供上位机直接做正解/显示，无需再除减速比

// 共享内存布局：锁不放在这里 —— sem_com 是独立的 System V 信号量，用 key 标识，
// 不在共享内存块内部。
struct ArmSharedData 
{
  ArmCommand command;
  ArmStatus status;
};

// 布局自检（编译期）：上下位机必须严格一致，Python 侧 tools/*.py 里也有对应的 sizeof/offset 断言。
// 一旦这里报错，说明结构体改了，必须同步修改 tools/set_angle.py、tools/upper_sim.py，
// 并递增 ARM_SHM_MAGIC。
static_assert(sizeof(ArmCommand) == 40, "ArmCommand 大小变了：请同步 Python 脚本并递增 ARM_SHM_MAGIC");
static_assert(sizeof(ArmStatus) == 88, "ArmStatus 大小变了：请同步 Python 脚本并递增 ARM_SHM_MAGIC");
static_assert(sizeof(ArmSharedData) == 128, "ArmSharedData 大小变了：请同步 Python 脚本并递增 ARM_SHM_MAGIC");
static_assert(offsetof(ArmCommand, mode) == 8, "ArmCommand.mode 偏移变了");
static_assert(offsetof(ArmCommand, magic) == 12, "ArmCommand.magic 偏移变了");
static_assert(offsetof(ArmCommand, theta) == 16, "ArmCommand.theta 偏移变了");
static_assert(offsetof(ArmStatus, motor_pos) == 40, "ArmStatus.motor_pos 偏移变了");
static_assert(offsetof(ArmStatus, joint_theta) == 64, "ArmStatus.joint_theta 偏移变了");

/*!
 * 基于 POSIX shm_open 的共享内存对象模板（仿照原 SharedMemory.h）。
 * 用名字字符串标识，可被多个进程同时映射。
 * createNew 分配新内存，attach 附加已存在的对象。
 */
template <typename T>
class SharedMemoryObject 
{
 public:
  SharedMemoryObject() = default;
  ~SharedMemoryObject() 
  {
    if (_data) detach();
  }

  /*!
   * 分配共享内存并映射。若同名对象已存在，allowOverwrite 为 false 时抛异常。
   */
  bool createNew(const std::string& name, bool allowOverwrite = false) 
  {
    bool hadToDelete = false;
    assert(!_data);
    _name = name;
    _size = sizeof(T);
    printf("[Shared Memory] open new %s, size %zu bytes\n", name.c_str(), _size);

    _fd = shm_open(name.c_str(), O_RDWR | O_CREAT, 0666);
    if (_fd == -1)
    {
      printf("[ERROR] SharedMemoryObject shm_open failed: %s\n", strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }
    // 上下位机可能以不同用户运行（如底层 root、上位机普通用户）。
    // shm_open 的模式会被进程 umask 掩码，这里用 fchmod 固定为 0666（不受 umask 影响），
    // 保证双方都能读写该共享内存。
    if (fchmod(_fd, 0666) == -1)
      printf("[ERROR] SharedMemoryObject fchmod(%s) failed: %s\n",
             name.c_str(), strerror(errno));

    struct stat s;
    if (fstat(_fd, &s)) 
    {
      printf("[ERROR] SharedMemoryObject::createNew(%s) stat: %s\n",
             name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    if (s.st_size) 
    {
      printf("[Shared Memory] SharedMemoryObject::createNew(%s) on something "
             "that wasn't new (size is %ld bytes)\n",
             _name.c_str(), s.st_size);
      hadToDelete = true;
      if (!allowOverwrite)
        throw std::runtime_error(
            "Failed to create shared memory - it already exists.");
      printf("\tusing existing shared memory!\n");
    }

    if (ftruncate(_fd, _size)) 
    {
      printf("[ERROR] SharedMemoryObject::createNew(%s) ftruncate(%zu): %s\n",
             name.c_str(), _size, strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    void* mem =
        mmap(nullptr, _size, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
    if (mem == MAP_FAILED) 
    {
      printf("[ERROR] SharedMemory::createNew(%s) mmap fail: %s\n",
             _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    // 复用旧内存时可能不是全零，统一清零，避免布局变化导致脏数据
    memset(mem, 0, _size);

    _data = (T*)mem;
    return hadToDelete;
  }

  /*!
   * 附加到一个已存在的共享内存对象。
   */
  void attach(const std::string& name) 
  {
    assert(!_data);
    _name = name;
    _size = sizeof(T);
    printf("[Shared Memory] open existing %s size %zu bytes\n", name.c_str(),
           _size);
    _fd = shm_open(name.c_str(), O_RDWR,
                   S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH);
    if (_fd == -1)
    {
      // 共享内存尚不存在（首次启动或已被删除）——预期情况，connect() 会转而 createNew 新建；
      // 这里用黄色告警而非红色错误，避免误导。
      printf_color(PrintColor::Yellow,
                   "[WARN] SharedMemoryObject::attach %s 失败(%s)，将尝试新建\n",
                   _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    struct stat s;
    if (fstat(_fd, &s)) 
    {
      printf("[ERROR] SharedMemoryObject::attach(%s) stat: %s\n", name.c_str(),
             strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    if ((size_t)s.st_size != _size) 
    {
      printf("[ERROR] SharedMemoryObject::attach(%s) on something that was "
             "incorrectly sized (size is %ld bytes, should be %zu)\n",
             _name.c_str(), s.st_size, _size);
      throw std::runtime_error("Failed to create shared memory!");
    }

    void* mem =
        mmap(nullptr, _size, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
    if (mem == MAP_FAILED) 
    {
      printf("[ERROR] SharedMemory::attach(%s) mmap fail: %s\n", _name.c_str(),
             strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    _data = (T*)mem;
  }

  /*!
   * 关闭当前进程对共享内存的映射（不删除共享内存，其他进程仍可用）。
   */
  void detach() 
  {
    if (!_data) return;

    if (munmap((void*)_data, _size)) 
    {
      printf("[ERROR] SharedMemoryObject::detach (%s) munmap %s\n",
             _name.c_str(), strerror(errno));
    }
    _data = nullptr;

    if (_fd >= 0) 
    {
      if (close(_fd)) 
      {
        printf("[ERROR] SharedMemoryObject::detach (%s) close %s\n",
               _name.c_str(), strerror(errno));
      }
      _fd = -1;
    }
  }

  /*!
   * 获取共享内存对象指针。
   */
  T* get() 
  {
    assert(_data);
    return _data;
  }

  /*!
   * 获取共享内存对象引用。
   */
  T& operator()() 
  {
    assert(_data);
    return *_data;
  }

 private:
  T* _data = nullptr;
  std::string _name;
  size_t _size = 0;
  int _fd = -1;
};

/*!
 * 上下位机共享内存访问封装：内部持有 System V 信号量锁（sem_com），
 * 每次读写 command/status 前 sem_p() 加锁、之后 sem_v() 解锁。
 */
class ArmSharedMemory 
{
 public:
  // 构造时打开互斥锁（首次创建者初始化为 1，其余进程打开现有锁）
  ArmSharedMemory() : lock_(ARM_SEM_KEY) {}

  // 连接共享内存：不存在则创建、已存在则附加，**并且每次启动都把整块清零**。
  // 清零是刻意的安全设计：
  //   ① 上一次运行可能遗留 mode=1 + 某组角度，接上电机再启动时会立刻按旧角度动作；
  //   ② 旧版本/外来程序留下的同尺寸数据字段含义可能不同（例如旧 x/y/z 被当成角度）；
  //   ③ 清零后 command.mode=0，底层在收到上位机的新命令前不会下发任何指令。
  bool connect(const std::string& name = ARM_SHARED_MEMORY_NAME) {
    try {
      obj_.attach(name);  // 已存在则附加
    } catch (const std::exception&) {
      // allowOverwrite=true：即使已有同名对象（甚至尺寸不同）也复用它并 ftruncate 到正确大小
      obj_.createNew(name, true);
    }

    ArmSharedData* d = obj_.get();
    const uint32_t prev_magic = d->command.magic;
    const uint8_t  prev_mode  = d->command.mode;

    *d = ArmSharedData{};   // 值初始化：command/status 全部归零，command.magic 恢复为默认魔数

    if (prev_magic != ARM_SHM_MAGIC && prev_magic != 0) {
      printf_color(PrintColor::Yellow,
                   "[Shared Memory] 检测到旧/外来数据（magic=0x%08X ≠ 0x%08X），已清零\n",
                   prev_magic, ARM_SHM_MAGIC);
    } else if (prev_mode != 0) {
      printf_color(PrintColor::Yellow,
                   "[Shared Memory] 上次遗留 mode=%u（可能带着旧角度命令），已清零\n", prev_mode);
    } else {
      printf_color(PrintColor::Green, "[Shared Memory] 已清零，等待上位机写入命令\n");
    }
    return true;
  }

  bool readCommand(ArmCommand& out) {
    lock_.sem_p();
    out = obj_.get()->command;
    lock_.sem_v();
    return true;
  }

  bool writeCommand(const ArmCommand& c) {
    lock_.sem_p();
    obj_.get()->command = c;
    lock_.sem_v();
    return true;
  }

  bool readStatus(ArmStatus& out) {
    lock_.sem_p();
    out = obj_.get()->status;
    lock_.sem_v();
    return true;
  }

  bool writeStatus(const ArmStatus& s) {
    lock_.sem_p();
    obj_.get()->status = s;
    lock_.sem_v();
    return true;
  }

 private:
  SharedMemoryObject<ArmSharedData> obj_;
  sem_com lock_;  // System V 信号量互斥锁（key = ARM_SEM_KEY）
};

#endif  // PROJECT_SHARED_MEMORY_H
