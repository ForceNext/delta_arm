#include "Kinematics/shm_client.hpp"

#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/sem.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace delta_arm
{
namespace
{
// 用系统自带的 struct sembuf / SEM_UNDO / IPC_CREAT（与 delta_bottom 的 sem_com.h 等价）
int create_or_open_sem(int key)
{
  int semid = ::semget(key, 1, IPC_CREAT | IPC_EXCL | 0666);
  if (semid >= 0) {
    // 首次创建：初始化为 1（互斥锁）
    if (::semctl(semid, 0, SETVAL, 1) < 0) {
      throw std::runtime_error(std::string("semctl(SETVAL) 失败: ") + std::strerror(errno));
    }
    return semid;
  }
  if (errno != EEXIST) {
    throw std::runtime_error(std::string("semget 失败: ") + std::strerror(errno));
  }
  semid = ::semget(key, 1, 0);
  if (semid < 0) {
    throw std::runtime_error(std::string("semget(打开) 失败: ") + std::strerror(errno));
  }
  return semid;
}
}  // namespace

ShmClient::ShmClient(std::string name, int sem_key)
: name_(std::move(name)), sem_key_(sem_key)
{
}

ShmClient::~ShmClient()
{
  if (data_ != nullptr) {
    ::munmap(data_, sizeof(ArmSharedData));
    data_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  // 信号量是全局命名对象，按 delta_bottom 的约定不在这里删除
}

bool ShmClient::connect()
{
  if (data_ != nullptr) {
    return true;
  }
  try {
    semid_ = create_or_open_sem(sem_key_);
  } catch (const std::exception &) {
    return false;
  }

  fd_ = ::shm_open(name_.c_str(), O_RDWR | O_CREAT, 0666);
  if (fd_ < 0) {
    return false;
  }
  struct stat st {};
  if (::fstat(fd_, &st) == 0 && st.st_size < static_cast<off_t>(sizeof(ArmSharedData))) {
    if (::ftruncate(fd_, sizeof(ArmSharedData)) != 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
  }

  void * p = ::mmap(nullptr, sizeof(ArmSharedData), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (p == MAP_FAILED) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  data_ = static_cast<ArmSharedData *>(p);

  // 首次创建（内容全 0）时写入魔数，声明布局版本；已有数据则不动
  if (data_->command.magic != kShmMagic && data_->command.seq == 0) {
    lock();
    data_->command.magic = kShmMagic;
    unlock();
  }
  return true;
}

bool ShmClient::lock()
{
  if (semid_ < 0) {
    return false;
  }
  sembuf op {0, -1, SEM_UNDO};
  return ::semop(semid_, &op, 1) == 0;
}

void ShmClient::unlock()
{
  if (semid_ < 0) {
    return;
  }
  sembuf op {0, 1, SEM_UNDO};
  (void)::semop(semid_, &op, 1);
}

bool ShmClient::writeCommand(uint8_t mode, const double theta_deg[3])
{
  if (data_ == nullptr || !lock()) {
    return false;
  }
  data_->command.magic = kShmMagic;
  data_->command.mode = mode;
  data_->command.seq += 1;
  for (int i = 0; i < 3; ++i) {
    data_->command.theta[i] = theta_deg[i];
  }
  unlock();
  return true;
}

bool ShmClient::readStatus(ArmStatus & out)
{
  if (data_ == nullptr || !lock()) {
    return false;
  }
  out = data_->status;
  unlock();
  return true;
}

}  // namespace delta_arm
