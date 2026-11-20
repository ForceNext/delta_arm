#ifndef DELTA_ARM_KINEMATICS_IK_HPP
#define DELTA_ARM_KINEMATICS_IK_HPP

/**
 * @file Kinematics_ik.hpp
 * @brief Delta 并联机械臂的闭式正解 / 逆解（纯数学，无 ROS 依赖）
 *
 * 坐标与角度约定：
 *   - 末端坐标 (x, y, z)，单位 mm；
 *     **z 向下为正**：z>0 表示动平台在基座下方，z=0 在肩关节轴线平面上；
 *     典型工作空间 z ∈ [138.5, 300] mm（z 越大越深）；
 *   - x 轴指向 1 号臂方位，2/3 号臂在 120° / 240°（绕 +z 逆时针，右手系 x×y=z）；
 *   - 关节角 θ，单位度：θ=0 表示上臂水平朝外，θ>0 表示上臂向下压；
 *   - 逆解取「肘向外下」的标准工作位形（另一组解在 acos 前把 - 换成 +）。
 *   注：内部推导沿用教科书"z 向上为正"的形式，在 inverse/forward 的入口出口各做一次 z 取反。
 *
 * 与下层的关系：逆解输出的关节角直接写进共享内存的 ArmCommand.theta，
 * 由 delta_bottom 负责「限幅 → 减 joint.offset → × joint.dir → × 减速比 → 计数」。
 * 因此本模块不做减速比/方向/偏置换算。
 */
namespace delta_arm
{

class DeltaKinematics
{
public:
  /**
   * @param d1 基座半径（mm，中心到肩关节轴）
   * @param L1 上臂长（mm，肩关节到肘关节）
   * @param L2 前臂/平行四边形杆长（mm，肘关节到动平台铰点）
   * @param d2 动平台半径（mm，中心到前臂铰点）
   */
  DeltaKinematics(double d1, double L1, double L2, double d2);

  /**
   * @brief 逆解：末端坐标 → 三个关节角
   * @param x,y,z 末端坐标（mm，**z 向下为正**）
   * @param theta_deg 出参：三个关节角（度）
   * @return 目标点在工作空间内返回 true；越界（acos 参数超出 [-1,1]）返回 false，不写出参
   */
  bool inverse(double x, double y, double z, double theta_deg[3]) const;

  /**
   * @brief 正解：三个关节角 → 末端坐标
   * @param theta_deg 三个关节角（度）
   * @param xyz 出参：末端坐标（mm，**z 向下为正**）
   * @return 成功返回 true；三球无交或退化（肘点近共线）返回 false
   */
  bool forward(const double theta_deg[3], double xyz[3]) const;

  // 几何参数只读访问（方便打日志/自检）
  double d1() const { return d1_; }
  double L1() const { return L1_; }
  double L2() const { return L2_; }
  double d2() const { return d2_; }

private:
  double d1_, L1_, L2_, d2_;
};

}  // namespace delta_arm

#endif  // DELTA_ARM_KINEMATICS_IK_HPP
