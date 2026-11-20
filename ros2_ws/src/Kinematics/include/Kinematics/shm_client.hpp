#ifndef DELTA_ARM_SHM_CLIENT_HPP
#define DELTA_ARM_SHM_CLIENT_HPP

/**
 * @file shm_client.hpp
 * @brief 上位机侧的共享内存客户端：与 delta_bottom 的 /delta_arm 契约严格一致
 *
 * 布局（必须与 delta_bottom/common/include/Utilities/SharedMemory.h 保持逐字节一致）：
 *   ArmCommand{ seq, mode, reserved[3], magic, theta[3] }  = 40 字节
 *   ArmStatus { seq, state, online[3], theta[3], motor_pos[3], joint_theta[3] } = 88 字节
 *   ArmSharedData{ command, status }                       = 128 字节
 * 互斥：System V 信号量，key 0x5A11（= delta_bottom 的 ARM_SEM_KEY）。
 *
 * 注意：delta_bottom **每次启动都会把整块清零**，所以本客户端要么在底层启动之后再写，
 * 要么持续写入（例如按固定频率重发当前目标）。
 */

#include <cstdint>
#include <string>

namespace delta_arm
{

constexpr uint32_t kShmMagic = 0x444C5431u;   // 'D''L''T''1'，与底层一致
constexpr int kShmSemKey = 0x5A11;            // System V 信号量 key
constexpr const char * kShmDefaultName = "/delta_arm";

struct ArmCommand
{
  uint64_t seq = 0;                    // 写入序号（每次写 +1）
  uint8_t mode = 0;                    // 0=空闲  1=按 theta 下发
  uint8_t reserved[3] = {0, 0, 0};     // 对齐填充
  uint32_t magic = kShmMagic;          // 布局魔数
  double theta[3] = {0.0, 0.0, 0.0};   // 三个关节角（度）
};

struct ArmStatus
{
  uint64_t seq = 0;                          // 底层写入序号（每拍递增，可判活）
  uint8_t state = 0;                         // bit0=正在按命令运行；bit1=实测有效
  uint8_t online[3] = {0, 0, 0};             // 各电机在线标志
  double theta[3] = {0.0, 0.0, 0.0};         // 电机角（度）——底层尚未回读时为 0
  double motor_pos[3] = {0.0, 0.0, 0.0};     // 电机计数   ——同上
  double joint_theta[3] = {0.0, 0.0, 0.0};   // 关节角（度）——同上
};

struct ArmSharedData
{
  ArmCommand command;
  ArmStatus status;
};

static_assert(sizeof(ArmCommand) == 40, "ArmCommand 布局变化，需与 delta_bottom 同步");
static_assert(sizeof(ArmStatus) == 88, "ArmStatus 布局变化，需与 delta_bottom 同步");
static_assert(sizeof(ArmSharedData) == 128, "ArmSharedData 必须为 128 字节");

class ShmClient
{
public:
  explicit ShmClient(std::string name = kShmDefaultName, int sem_key = kShmSemKey);
  ~ShmClient();

  ShmClient(const ShmClient &) = delete;
  ShmClient & operator=(const ShmClient &) = delete;

  /** 打开（不存在则创建）共享内存与信号量；成功返回 true */
  bool connect();

  /** 写命令块（加锁）：mode + 三个关节角，seq 自增 */
  bool writeCommand(uint8_t mode, const double theta_deg[3]);

  /** 读状态块（加锁） */
  bool readStatus(ArmStatus & out);

  bool connected() const { return data_ != nullptr; }

private:
  bool lock();
  void unlock();

  std::string name_;
  int sem_key_;
  int semid_ = -1;
  int fd_ = -1;
  ArmSharedData * data_ = nullptr;
};

}  // namespace delta_arm

#endif  // DELTA_ARM_SHM_CLIENT_HPP
