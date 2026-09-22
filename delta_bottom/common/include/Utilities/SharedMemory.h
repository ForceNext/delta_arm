/*! @file SharedMemory.h
 *  @brief delta_arm 上下位机共享内存
 *
 *  基于 POSIX shm_open 的 SharedMemoryObject 模板，
 *  读写互斥锁用 System V 信号量 sem_com（Utilities/sem_com.h）管理。
 *
 *  底层（delta_bottom）与上位机共用同一块共享内存：
 *    上位机写入 ArmCommand，底层读取并下发。
 *
 *  当前阶段（暂不做运动学解算）：
 *    x -> 电机 id1 目标角度（度）
 *    y -> 电机 id2 目标角度（度）
 *    z -> 电机 id3 目标角度（度）
 */
#ifndef PROJECT_SHARED_MEMORY_H
#define PROJECT_SHARED_MEMORY_H

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cassert>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include "Utilities/sem_com.h"

// 共享内存名称（shm_open 的名字须以 '/' 开头）
#define ARM_SHARED_MEMORY_NAME "/delta_arm"

// System V 信号量 key（上下位机必须使用同一个 key，任意非零整数，避免与其他程序冲突）
#define ARM_SEM_KEY 0x5A11

// 上位机 -> 底层：命令（xyz 直接作为三个电机的目标角度，单位：度）
struct ArmCommand {
  uint64_t seq = 0;   // 写入序号（每次写 +1）
  uint8_t mode = 0;   // 0=空闲  1=位置模式（把 xyz 当角度下发）
  double x = 0.0;     // 电机 id1 目标角度（度）
  double y = 0.0;     // 电机 id2 目标角度（度）
  double z = 0.0;     // 电机 id3 目标角度（度）
};

// 底层 -> 上位机：状态
struct ArmStatus {
  uint64_t seq = 0;                  // 写入序号
  uint8_t state = 0;                 // 0=空闲 1=运行
  uint8_t online[3] = {0, 0, 0};     // 各电机在线标志
  double theta[3] = {0, 0, 0};       // 实际下发的三个目标角度（度）
  double motor_pos[3] = {0, 0, 0};   // 实际下发的电机位置（计数）
  double x = 0.0, y = 0.0, z = 0.0;  // 回显坐标
};

// 共享内存布局：锁不放在这里 —— sem_com 是独立的 System V 信号量，用 key 标识，
// 不在共享内存块内部。
struct ArmSharedData {
  ArmCommand command;
  ArmStatus status;
};

/*!
 * 基于 POSIX shm_open 的共享内存对象模板（仿照原 SharedMemory.h）。
 * 用名字字符串标识，可被多个进程同时映射。
 * createNew 分配新内存，attach 附加已存在的对象。
 */
template <typename T>
class SharedMemoryObject {
 public:
  SharedMemoryObject() = default;
  ~SharedMemoryObject() {
    if (_data) detach();
  }

  /*!
   * 分配共享内存并映射。若同名对象已存在，allowOverwrite 为 false 时抛异常。
   */
  bool createNew(const std::string& name, bool allowOverwrite = false) {
    bool hadToDelete = false;
    assert(!_data);
    _name = name;
    _size = sizeof(T);
    printf("[Shared Memory] open new %s, size %zu bytes\n", name.c_str(), _size);

    _fd = shm_open(name.c_str(), O_RDWR | O_CREAT,
                   S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH);
    if (_fd == -1) {
      printf("[ERROR] SharedMemoryObject shm_open failed: %s\n", strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    struct stat s;
    if (fstat(_fd, &s)) {
      printf("[ERROR] SharedMemoryObject::createNew(%s) stat: %s\n",
             name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    if (s.st_size) {
      printf("[Shared Memory] SharedMemoryObject::createNew(%s) on something "
             "that wasn't new (size is %ld bytes)\n",
             _name.c_str(), s.st_size);
      hadToDelete = true;
      if (!allowOverwrite)
        throw std::runtime_error(
            "Failed to create shared memory - it already exists.");
      printf("\tusing existing shared memory!\n");
    }

    if (ftruncate(_fd, _size)) {
      printf("[ERROR] SharedMemoryObject::createNew(%s) ftruncate(%zu): %s\n",
             name.c_str(), _size, strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    void* mem =
        mmap(nullptr, _size, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
    if (mem == MAP_FAILED) {
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
  void attach(const std::string& name) {
    assert(!_data);
    _name = name;
    _size = sizeof(T);
    printf("[Shared Memory] open existing %s size %zu bytes\n", name.c_str(),
           _size);
    _fd = shm_open(name.c_str(), O_RDWR,
                   S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH);
    if (_fd == -1) {
      printf("[ERROR] SharedMemoryObject::attach shm_open(%s) failed: %s\n",
             _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    struct stat s;
    if (fstat(_fd, &s)) {
      printf("[ERROR] SharedMemoryObject::attach(%s) stat: %s\n", name.c_str(),
             strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    if ((size_t)s.st_size != _size) {
      printf("[ERROR] SharedMemoryObject::attach(%s) on something that was "
             "incorrectly sized (size is %ld bytes, should be %zu)\n",
             _name.c_str(), s.st_size, _size);
      throw std::runtime_error("Failed to create shared memory!");
    }

    void* mem =
        mmap(nullptr, _size, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
    if (mem == MAP_FAILED) {
      printf("[ERROR] SharedMemory::attach(%s) mmap fail: %s\n", _name.c_str(),
             strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
    }

    _data = (T*)mem;
  }

  /*!
   * 关闭当前进程对共享内存的映射（不删除共享内存，其他进程仍可用）。
   */
  void detach() {
    if (!_data) return;

    if (munmap((void*)_data, _size)) {
      printf("[ERROR] SharedMemoryObject::detach (%s) munmap %s\n",
             _name.c_str(), strerror(errno));
    }
    _data = nullptr;

    if (_fd >= 0) {
      if (close(_fd)) {
        printf("[ERROR] SharedMemoryObject::detach (%s) close %s\n",
               _name.c_str(), strerror(errno));
      }
      _fd = -1;
    }
  }

  /*!
   * 获取共享内存对象指针。
   */
  T* get() {
    assert(_data);
    return _data;
  }

  /*!
   * 获取共享内存对象引用。
   */
  T& operator()() {
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
class ArmSharedMemory {
 public:
  // 构造时打开互斥锁（首次创建者初始化为 1，其余进程打开现有锁）
  ArmSharedMemory() : lock_(ARM_SEM_KEY) {}

  // 连接共享内存：首次启动的进程创建，其余附加
  bool connect(const std::string& name = ARM_SHARED_MEMORY_NAME) {
    try {
      obj_.attach(name);  // 已存在则附加
    } catch (const std::exception&) {
      obj_.createNew(name);  // 不存在则创建
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
