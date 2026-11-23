/**
 * @file Kinematics_node.cpp
 * @brief 订阅末端目标 (x, y, z) → 逆解 → 写共享内存 → delta_bottom 驱动电机
 *
 * 数据流：
 *   /delta_arm/target_xyz (geometry_msgs/Point)
 *        → DeltaKinematics::inverse()        （xyz mm → 关节角 deg）
 *        → 关节限幅（joint_min / joint_max）
 *        → ShmClient::writeCommand(mode=1)   （共享内存 /delta_arm，信号量 0x5A11）
 *        → delta_bottom：限幅 → 减 offset → × dir → × 减速比 → 0xF2 下发
 *
 * 关键提醒：
 *   1) delta_bottom 每次启动都会清零共享内存，所以启动顺序是「先起底层，再起本节点/开始发目标」，
 *      或者让发布端持续以固定频率重发目标值；
 *   2) theta 是**关节角**，而底层的 `joint.offset` 决定"哪个物理姿态对应哪个角度"，
 *      两边必须一致（用 tools/calib_offset.py 标定）；
 *   3) 不可达（IK 返回 false）时不写入，保持上一次有效目标，避免下发垃圾角度。
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "Kinematics/Kinematics_ik.hpp"
#include "Kinematics/shm_client.hpp"

using namespace std::chrono_literals;

class KinematicsNode : public rclcpp::Node
{
public:
  KinematicsNode()
  : Node("delta_kinematics")
  {
    // —— 机构几何（mm）：默认值取自 hardware_config.yaml 的 joint.jointlength ——
    const double d1 = this->declare_parameter("d1", 115.0);
    const double L1 = this->declare_parameter("L1", 150.0);
    const double L2 = this->declare_parameter("L2", 292.0);
    const double d2 = this->declare_parameter("d2", 40.0);
    ik_ = std::make_unique<delta_arm::DeltaKinematics>(d1, L1, L2, d2);

    // —— 关节限幅（度）：必须与 hardware_config.yaml 的 joint.min_pos / max_pos 一致 ——
    jmin_ = this->declare_parameter<std::vector<double>>(
      "joint_min", std::vector<double>{-25.0, -25.0, -25.0});   // 与 hardware_config.yaml 同步
    jmax_ = this->declare_parameter<std::vector<double>>(
      "joint_max", std::vector<double>{70.0, 70.0, 70.0});   // 与 hardware_config.yaml 的 max_pos 同步
    if (jmin_.size() != 3 || jmax_.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "joint_min / joint_max 必须是 3 个数，改用默认 [-25, 70]");
      jmin_ = {-25.0, -25.0, -25.0};
      jmax_ = {70.0, 70.0, 70.0};
    }

    // —— 其它参数 ——
    topic_ = this->declare_parameter<std::string>("xyz_topic", "/delta_arm/target_xyz");
    js_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/delta_arm/joint_states");
    actual_topic_ = this->declare_parameter<std::string>("actual_xyz_topic", "/delta_arm/actual_xyz");
    shm_name_ = this->declare_parameter<std::string>("shm_name", delta_arm::kShmDefaultName);
    sem_key_ = this->declare_parameter("sem_key", delta_arm::kShmSemKey);
    dry_run_ = this->declare_parameter("dry_run", false);
    timeout_s_ = this->declare_parameter("timeout_s", 0.0);   // >0：超过这么久没收到目标就置空闲
    self_check_ = this->declare_parameter("self_check", true);
    // z 范围守卫（mm，**z 向下为正**，数值越大越深）：当前规定 z ∈ [138, 300]
    z_min_ = this->declare_parameter("z_min", 138.0);
    z_max_ = this->declare_parameter("z_max", 300.0);
    // 末端 xy 的圆形边界（mm）：r = √(x²+y²) 不得超过它；<=0 表示不启用
    radius_max_ = this->declare_parameter("radius_max", 150.0);
    reach_tol_ = this->declare_parameter("reach_tol_mm", 1.0);

    RCLCPP_INFO(
      this->get_logger(), "geometry: d1=%.1f L1=%.1f L2=%.1f d2=%.1f mm", d1, L1, L2, d2);
    RCLCPP_INFO(
      this->get_logger(), "limits: [%.1f, %.1f] [%.1f, %.1f] [%.1f, %.1f] deg",
      jmin_[0], jmax_[0], jmin_[1], jmax_[1], jmin_[2], jmax_[2]);

    // 由关节限幅推出对称轴上的 z 参考点（z 向下为正 → 数值越大越深）
    const double th_low[3] = {jmin_[0], jmin_[1], jmin_[2]};
    const double th_high[3] = {jmax_[0], jmax_[1], jmax_[2]};
    const double th_zero[3] = {0.0, 0.0, 0.0};
    double z_at_jmin = 0.0, z_at_jmax = 0.0, z_at_zero = 0.0, tmp[3] = {0, 0, 0};
    if (ik_->forward(th_low, tmp)) { z_at_jmin = tmp[2]; }
    if (ik_->forward(th_high, tmp)) { z_at_jmax = tmp[2]; }
    if (ik_->forward(th_zero, tmp)) { z_at_zero = tmp[2]; }

    if (std::isnan(z_min_)) {
      z_min_ = z_at_jmin;                       // 传 NaN 时才自动用 θ(joint_min) 的位置
    }
    RCLCPP_INFO(
      this->get_logger(),
      "z 向下为正：z(θ=%+.0f°)=%.3f  z(θ=0°)=%.3f  z(θ=%+.0f°)=%.3f mm", jmin_[0],
      z_at_jmin, z_at_zero, jmax_[0], z_at_jmax);
    RCLCPP_INFO(
      this->get_logger(), "规定 z ∈ [%.3f, %.3f] mm（向下为正；可用 -p z_min/z_max 覆盖）", z_min_,
      z_max_);
    if (radius_max_ > 0.0) {
      RCLCPP_INFO(
        this->get_logger(), "末端圆形边界: r = √(x²+y²) ≤ %.1f mm（-p radius_max 可改）",
        radius_max_);
    } else {
      RCLCPP_INFO(this->get_logger(), "末端圆形边界: 未启用（radius_max<=0）");
    }

    shm_ = std::make_unique<delta_arm::ShmClient>(shm_name_, sem_key_);
    if (!shm_->connect()) {
      RCLCPP_ERROR(
        this->get_logger(), "打开共享内存 %s / 信号量 0x%X 失败", shm_name_.c_str(), sem_key_);
    } else {
      RCLCPP_INFO(
        this->get_logger(), "shared memory ready: %s (sem key 0x%X)", shm_name_.c_str(), sem_key_);
    }

    // 输出（话题）：解算出的关节角（弧度，JointState 标准）、以及夹取后正解得到的实际落点
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(js_topic_, rclcpp::QoS(10));
    actual_pub_ = this->create_publisher<geometry_msgs::msg::Point>(actual_topic_, rclcpp::QoS(10));

    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      topic_, rclcpp::QoS(10),
      std::bind(&KinematicsNode::onTarget, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscribed: %s", topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(), "publishing: %s (JointState, 弧度)  %s (Point, mm)",
      js_topic_.c_str(), actual_topic_.c_str());

    if (timeout_s_ > 0.0) {
      timer_ = this->create_wall_timer(100ms, std::bind(&KinematicsNode::onTimer, this));
    }
  }

  ~KinematicsNode() override
  {
    // 退出时置空闲，避免底层继续按最后一个目标保持/动作
    if (shm_ && shm_->connected() && !dry_run_) {
      const double idle[3] = {0.0, 0.0, 0.0};
      shm_->writeCommand(0, idle);
      RCLCPP_INFO(this->get_logger(), "shutdown: 已写 mode=0（空闲）");
    }
  }

private:
  void onTarget(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    double th[3] = {0.0, 0.0, 0.0};
    if (!ik_->inverse(msg->x, msg->y, msg->z, th)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "逆解不可达 (%.1f, %.1f, %.1f) mm，本次不下发（保持上一次目标）",
        msg->x, msg->y, msg->z);
      return;
    }

    // ① z 范围守卫（mm，向下为正：太小=太浅，太大=太深）
    if (msg->z < z_min_ || msg->z > z_max_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "目标 z=%.2f mm 超出允许范围 [%.2f, %.2f]（向下为正），本次不下发", msg->z, z_min_,
        z_max_);
      return;
    }

    // ② 末端 xy 圆形边界
    if (radius_max_ > 0.0) {
      const double r = std::hypot(msg->x, msg->y);
      if (r > radius_max_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "目标半径 r=%.2f mm 超出圆形边界 %.1f mm（x=%.1f, y=%.1f），本次不下发", r, radius_max_,
          msg->x, msg->y);
        return;
      }
    }

    // ③ 关节限幅
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

    // ④ 夹取后回代正解：若夹完已经到不了请求点，说明目标在允许工作空间之外 → 不下发
    //    （否则底层会把"夹过的角度"当目标，实际位置和请求位置就不是同一个点了）
    double reached[3] = {0.0, 0.0, 0.0};
    if (clamped) {
      const bool okf = ik_->forward(th, reached);
      const double dev = okf ? std::sqrt(
        (reached[0] - msg->x) * (reached[0] - msg->x) +
        (reached[1] - msg->y) * (reached[1] - msg->y) +
        (reached[2] - msg->z) * (reached[2] - msg->z)) : 1e9;
      if (!okf || dev > reach_tol_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "目标 (%.1f, %.1f, %.1f) 超出关节限幅对应的可达空间（夹到 θ=(%.2f, %.2f, %.2f) 后"
          "只能到 (%.1f, %.1f, %.1f)，偏差 %.1f mm），本次不下发",
          msg->x, msg->y, msg->z, th[0], th[1], th[2],
          reached[0], reached[1], reached[2], dev);
        return;
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "逆解结果超出限幅，已夹到 θ=(%.2f, %.2f, %.2f) deg（落点偏差 %.2f mm）",
        th[0], th[1], th[2], dev);
    }

    // 一次性自检：θ → 正解 → 与目标比较，验证几何参数与正逆解一致
    if (self_check_ && !checked_) {
      checked_ = true;
      double xyz[3] = {0.0, 0.0, 0.0};
      if (ik_->forward(th, xyz)) {
        const double err = std::sqrt(
          (xyz[0] - msg->x) * (xyz[0] - msg->x) + (xyz[1] - msg->y) * (xyz[1] - msg->y) +
          (xyz[2] - msg->z) * (xyz[2] - msg->z));
        RCLCPP_INFO(
          this->get_logger(), "自检 FK(IK) 残差 %.4f mm（目标 %.1f, %.1f, %.1f → 正解 %.2f, %.2f, %.2f）",
          err, msg->x, msg->y, msg->z, xyz[0], xyz[1], xyz[2]);
      }
    }

    if (!dry_run_ && shm_ && shm_->connected()) {
      if (!shm_->writeCommand(1, th)) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "写共享内存失败（信号量/映射异常）");
      }
    }

    // 输出话题 ①：关节角（JointState 标准用弧度）
    {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.name = {"joint1", "joint2", "joint3"};
      js.position.resize(3);
      js.velocity.resize(3, 0.0);
      js.effort.resize(3, 0.0);
      for (int i = 0; i < 3; ++i) {
        js.position[i] = th[i] * M_PI / 180.0;
      }
      js_pub_->publish(js);
    }

    // 输出话题 ②：夹取后正解的实际落点（和请求值对比就知道差多少）
    {
      geometry_msgs::msg::Point p;
      double f[3] = {0.0, 0.0, 0.0};
      if (ik_->forward(th, f)) {
        p.x = f[0];
        p.y = f[1];
        p.z = f[2];
      } else {
        p.x = p.y = p.z = std::nan("");
      }
      actual_pub_->publish(p);
    }

    last_msg_time_ = this->now();
    have_target_ = true;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "xyz(%.1f, %.1f, %.1f) → θ(%.2f, %.2f, %.2f) deg%s",
      msg->x, msg->y, msg->z, th[0], th[1], th[2], dry_run_ ? "  [dry-run]" : "");
  }

  void onTimer()
  {
    if (!have_target_ || !shm_ || !shm_->connected()) {
      return;
    }
    const double dt = (this->now() - last_msg_time_).seconds();
    if (dt > timeout_s_) {
      const double idle[3] = {0.0, 0.0, 0.0};
      shm_->writeCommand(0, idle);
      have_target_ = false;
      RCLCPP_WARN(
        this->get_logger(), "%.2f s 未收到目标，已写 mode=0（空闲）", dt);
    }
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
  bool dry_run_ = false;
  bool self_check_ = true;
  bool checked_ = false;
  bool have_target_ = false;
  double timeout_s_ = 0.0;
  double z_min_ = std::nan("");
  double z_max_ = std::nan("");
  double reach_tol_ = 1.0;
  double radius_max_ = 150.0;
  rclcpp::Time last_msg_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
