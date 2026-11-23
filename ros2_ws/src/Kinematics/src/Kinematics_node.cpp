/**
 * @file Kinematics_node.cpp
 * @brief 订阅末端目标 (x, y, z) → 逆解 → 写共享内存 / 发布话题
 *
 * 数据流：
 *   /delta_arm/target_xyz (geometry_msgs/Point)          ← 目标点
 *        → 守卫：z 范围 / xy 圆形边界 / 关节限幅 / 夹取后回代校验
 *        → DeltaKinematics::inverse()                    （xyz mm → 关节角 deg）
 *        → ShmClient::writeCommand(mode=1)               （共享内存 /delta_arm）
 *        → delta_bottom：限幅 → 减 offset → × dir → × 减速比 → 0xF2 下发
 *   同时按 topic_rate_hz 发布 /delta_arm/joint_states (JointState, 弧度) 与
 *   /delta_arm/actual_xyz (Point, mm)。
 *
 * 说明：
 *   - 收到目标**直接下发**（不做末端插值）；
 *   - 节点默认绑在 CPU 3（delta_bottom 的 4 ms 实时线程在 CPU 4）；
 *   - delta_bottom 每次启动都会清零共享内存，所以「先起底层，再起本节点/开始发目标」；
 *   - theta 是**关节角**，底层 `joint.offset` 决定"哪个物理姿态对应哪个角度"，两边必须一致。
 */

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "Kinematics/Kinematics_ik.hpp"
#include "Kinematics/shm_client.hpp"

class KinematicsNode : public rclcpp::Node
{
public:
  KinematicsNode()
  : Node("delta_kinematics")
  {
    // —— CPU 亲和性：默认绑到另一个核（delta_bottom 的 4ms 实时线程在 CPU 4）——
    cpu_affinity_ = this->declare_parameter("cpu_affinity", 3);
    if (cpu_affinity_ >= 0) {
      cpu_set_t set;
      CPU_ZERO(&set);
      CPU_SET(static_cast<size_t>(cpu_affinity_), &set);
      const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
      if (rc == 0) {
        RCLCPP_INFO(
          this->get_logger(), "CPU 亲和性: 绑定到 CPU %d（底层实时线程在 CPU 4）", cpu_affinity_);
      } else {
        RCLCPP_WARN(
          this->get_logger(), "绑定 CPU %d 失败（rc=%d），保持默认调度", cpu_affinity_, rc);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "CPU 亲和性: 未绑定（cpu_affinity<0）");
    }

    // —— 机构几何（mm）：默认值取自 hardware_config.yaml 的 joint.jointlength ——
    const double d1 = this->declare_parameter("d1", 115.0);
    const double L1 = this->declare_parameter("L1", 150.0);
    const double L2 = this->declare_parameter("L2", 292.0);
    const double d2 = this->declare_parameter("d2", 40.0);
    ik_ = std::make_unique<delta_arm::DeltaKinematics>(d1, L1, L2, d2);

    // —— 关节限幅（度）：必须与 hardware_config.yaml 的 joint.min_pos / max_pos 一致 ——
    jmin_ = this->declare_parameter<std::vector<double>>(
      "joint_min", std::vector<double>{-25.0, -25.0, -25.0});
    jmax_ = this->declare_parameter<std::vector<double>>(
      "joint_max", std::vector<double>{70.0, 70.0, 70.0});
    if (jmin_.size() != 3 || jmax_.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "joint_min / joint_max 必须是 3 个数，改用默认 [-25, 70]");
      jmin_ = {-25.0, -25.0, -25.0};
      jmax_ = {70.0, 70.0, 70.0};
    }

    // —— 末端工作空间守卫 ——
    z_min_ = this->declare_parameter("z_min", 138.0);
    z_max_ = this->declare_parameter("z_max", 300.0);
    radius_max_ = this->declare_parameter("radius_max", 150.0);   // <=0 关闭
    reach_tol_ = this->declare_parameter("reach_tol_mm", 1.0);

    // —— 其它 ——
    topic_ = this->declare_parameter<std::string>("xyz_topic", "/delta_arm/target_xyz");
    js_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/delta_arm/joint_states");
    actual_topic_ = this->declare_parameter<std::string>("actual_xyz_topic", "/delta_arm/actual_xyz");
    shm_name_ = this->declare_parameter<std::string>("shm_name", delta_arm::kShmDefaultName);
    sem_key_ = this->declare_parameter("sem_key", delta_arm::kShmSemKey);
    dry_run_ = this->declare_parameter("dry_run", false);
    timeout_s_ = this->declare_parameter("timeout_s", 0.0);
    self_check_ = this->declare_parameter("self_check", true);
    topic_rate_hz_ = this->declare_parameter("topic_rate_hz", 50.0);
    if (topic_rate_hz_ < 1.0) {
      topic_rate_hz_ = 50.0;
    }

    RCLCPP_INFO(
      this->get_logger(), "geometry: d1=%.1f L1=%.1f L2=%.1f d2=%.1f mm", d1, L1, L2, d2);
    RCLCPP_INFO(
      this->get_logger(), "limits: [%.1f, %.1f] [%.1f, %.1f] [%.1f, %.1f] deg",
      jmin_[0], jmax_[0], jmin_[1], jmax_[1], jmin_[2], jmax_[2]);
    RCLCPP_INFO(
      this->get_logger(), "z 向下为正，规定 z ∈ [%.3f, %.3f] mm", z_min_, z_max_);
    if (radius_max_ > 0.0) {
      RCLCPP_INFO(
        this->get_logger(), "末端圆形边界: r = √(x²+y²) ≤ %.1f mm（-p radius_max 可改）",
        radius_max_);
    } else {
      RCLCPP_INFO(this->get_logger(), "末端圆形边界: 未启用（radius_max<=0）");
    }
    RCLCPP_INFO(this->get_logger(), "状态发布: %.0f Hz（-p topic_rate_hz 可改）", topic_rate_hz_);

    shm_ = std::make_unique<delta_arm::ShmClient>(shm_name_, sem_key_);
    if (!shm_->connect()) {
      RCLCPP_ERROR(
        this->get_logger(), "打开共享内存 %s / 信号量 0x%X 失败", shm_name_.c_str(), sem_key_);
    } else {
      RCLCPP_INFO(
        this->get_logger(), "shared memory ready: %s (sem key 0x%X)", shm_name_.c_str(), sem_key_);
    }

    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(js_topic_, rclcpp::QoS(10));
    actual_pub_ = this->create_publisher<geometry_msgs::msg::Point>(actual_topic_, rclcpp::QoS(10));

    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      topic_, rclcpp::QoS(10),
      std::bind(&KinematicsNode::onTarget, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscribed: %s", topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(), "publishing: %s (JointState, 弧度)  %s (Point, mm)",
      js_topic_.c_str(), actual_topic_.c_str());

    // 状态发布 + 超时巡检（固定频率）
    const auto period = std::chrono::duration<double>(1.0 / topic_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&KinematicsNode::onTimer, this));
  }

  ~KinematicsNode() override
  {
    if (shm_ && shm_->connected() && !dry_run_) {
      const double idle[3] = {0.0, 0.0, 0.0};
      shm_->writeCommand(0, idle);
      RCLCPP_INFO(this->get_logger(), "shutdown: 已写 mode=0（空闲）");
    }
  }

private:
  // ---------- 收到目标：校验 → 逆解 → 下发 ----------
  void onTarget(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    const double x = msg->x, y = msg->y, z = msg->z;

    // ① z 范围
    if (z < z_min_ || z > z_max_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "目标被拒绝: z=%.2f mm 超出允许范围 [%.2f, %.2f]（向下为正）", z, z_min_, z_max_);
      return;
    }
    // ② xy 圆形边界
    if (radius_max_ > 0.0) {
      const double r = std::hypot(x, y);
      if (r > radius_max_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "目标被拒绝: 半径 r=%.2f mm 超出圆形边界 %.1f mm（x=%.1f, y=%.1f）", r, radius_max_, x, y);
        return;
      }
    }
    // ③ 逆解
    double th[3] = {0.0, 0.0, 0.0};
    if (!ik_->inverse(x, y, z, th)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "目标被拒绝: 逆解不可达 (%.1f, %.1f, %.1f) mm，保持上一次目标", x, y, z);
      return;
    }
    // ④ 关节限幅 + 夹取后回代校验
    bool clamped = false;
    for (int i = 0; i < 3; ++i) {
      if (th[i] < jmin_[i]) {
        th[i] = jmin_[i];
        clamped = true;
      } else if (th[i] > jmax_[i]) {
        th[i] = jmax_[i];
        clamped = true;
      }
    }
    if (clamped) {
      double f[3] = {0.0, 0.0, 0.0};
      const bool okf = ik_->forward(th, f);
      const double dev = okf ? std::sqrt(
        (f[0] - x) * (f[0] - x) + (f[1] - y) * (f[1] - y) + (f[2] - z) * (f[2] - z)) : 1e9;
      if (!okf || dev > reach_tol_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "目标被拒绝: (%.1f, %.1f, %.1f) 超出关节限幅对应的可达空间（夹到 θ=(%.2f,%.2f,%.2f) 只能到 "
          "(%.1f,%.1f,%.1f)，偏差 %.1f mm）",
          x, y, z, th[0], th[1], th[2], f[0], f[1], f[2], dev);
        return;
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "逆解超出限幅，已夹到 θ=(%.2f, %.2f, %.2f) deg（落点偏差 %.2f mm）", th[0], th[1], th[2], dev);
    }

    // 一次性自检：θ → 正解 → 与目标比较
    if (self_check_ && !checked_) {
      checked_ = true;
      double f[3] = {0.0, 0.0, 0.0};
      if (ik_->forward(th, f)) {
        const double err = std::sqrt(
          (f[0] - x) * (f[0] - x) + (f[1] - y) * (f[1] - y) + (f[2] - z) * (f[2] - z));
        RCLCPP_INFO(
          this->get_logger(), "自检 FK(IK) 残差 %.4f mm（目标 %.1f, %.1f, %.1f → 正解 %.2f,%.2f,%.2f）",
          err, x, y, z, f[0], f[1], f[2]);
      }
    }

    // ⑤ 写共享内存（底层每 4 ms 取一次最新值）
    if (!dry_run_ && shm_ && shm_->connected()) {
      if (!shm_->writeCommand(1, th)) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "写共享内存失败（信号量/映射异常）");
      }
    }

    for (int i = 0; i < 3; ++i) {
      last_th_[i] = th[i];
    }
    have_th_ = true;
    last_msg_time_ = this->now();
    idle_sent_ = false;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "cmd xyz(%.2f, %.2f, %.2f) → θ(%.2f, %.2f, %.2f) deg%s", x, y, z, th[0], th[1], th[2],
      dry_run_ ? "  [dry-run]" : "");
  }

  // ---------- 定时器：发布状态 + 超时置空闲 ----------
  void onTimer()
  {
    // 超时（可选）：长时间没有新目标 → 写 mode=0
    if (timeout_s_ > 0.0 && have_th_ && !idle_sent_ &&
      (this->now() - last_msg_time_).seconds() > timeout_s_)
    {
      if (shm_ && shm_->connected() && !dry_run_) {
        const double idle[3] = {0.0, 0.0, 0.0};
        shm_->writeCommand(0, idle);
        idle_sent_ = true;
        RCLCPP_WARN(this->get_logger(), "%.2f s 未收到目标，已写 mode=0（空闲）", timeout_s_);
      }
    }

    if (!have_th_) {
      return;
    }
    // ① 关节角（JointState 标准用弧度）
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = {"joint1", "joint2", "joint3"};
    js.position.resize(3);
    js.velocity.resize(3, 0.0);
    js.effort.resize(3, 0.0);
    for (int i = 0; i < 3; ++i) {
      js.position[i] = last_th_[i] * M_PI / 180.0;
    }
    js_pub_->publish(js);

    // ② 夹取后正解的实际落点
    geometry_msgs::msg::Point p;
    double f[3] = {0.0, 0.0, 0.0};
    if (ik_->forward(last_th_, f)) {
      p.x = f[0];
      p.y = f[1];
      p.z = f[2];
    } else {
      p.x = p.y = p.z = std::nan("");
    }
    actual_pub_->publish(p);
  }

  std::unique_ptr<delta_arm::DeltaKinematics> ik_;
  std::unique_ptr<delta_arm::ShmClient> shm_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr actual_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<double> jmin_, jmax_;
  std::string topic_, js_topic_, actual_topic_, shm_name_;
  int sem_key_ = delta_arm::kShmSemKey;
  int cpu_affinity_ = 3;
  bool dry_run_ = false;
  bool self_check_ = true;
  bool checked_ = false;
  double timeout_s_ = 0.0;
  double topic_rate_hz_ = 50.0;
  rclcpp::Time last_msg_time_;
  bool idle_sent_ = false;

  // 工作空间守卫
  double z_min_ = 138.0;
  double z_max_ = 300.0;
  double radius_max_ = 150.0;
  double reach_tol_ = 1.0;

  // 最近一次下发的角度（供状态话题）
  double last_th_[3] = {0.0, 0.0, 0.0};
  bool have_th_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
