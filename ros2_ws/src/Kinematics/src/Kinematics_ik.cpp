#include "Kinematics/Kinematics_ik.hpp"

#include <cmath>

namespace delta_arm
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
}  // namespace

DeltaKinematics::DeltaKinematics(double d1, double L1, double L2, double d2)
: d1_(d1), L1_(L1), L2_(L2), d2_(d2)
{
}

bool DeltaKinematics::inverse(double x, double y, double z_down, double theta_deg[3]) const
{
  const double z = -z_down;          // 对外 z 向下为正；内部推导用"向上为正"
  double th[3];
  for (int i = 0; i < 3; ++i) {
    const double phi = i * 120.0 * kDeg2Rad;      // 臂方位角 0/120/240°
    const double c = std::cos(phi);
    const double s = std::sin(phi);

    // 目标点投影到第 i 臂的竖直平面
    const double Drad = x * c + y * s - (d1_ - d2_);   // 径向分量
    const double Dtan = -x * s + y * c;                // 切向分量
    const double M = Drad * Drad + Dtan * Dtan + z * z + L1_ * L1_ - L2_ * L2_;
    const double Rh = std::sqrt(Drad * Drad + z * z);

    if (Rh < 1e-9) {
      return false;                                     // 目标点在竖直轴上，退化
    }
    const double val = M / (2.0 * L1_ * Rh);
    if (val < -1.0 || val > 1.0) {
      return false;                                     // 目标点超出工作空间
    }

    // 标准工作位形（肘向外下）；若实际机构方向相反，把下面的减号改成加号
    th[i] = (std::atan2(-z, Drad) - std::acos(val)) / kDeg2Rad;
  }
  // 自洽校验：把解代回正解，必须能回到同一个点。
  // 否则是"判别式合法但不在工作空间"的假解（例如基座上方 z>0 的点会解出 ~-216°），
  // 这种解一旦下发，下层限幅会把它夹到边界，造成一次大幅乱动。
  double back[3] = {0.0, 0.0, 0.0};
  if (!forward(th, back)) {
    return false;
  }
  const double dx = back[0] - x;
  const double dy = back[1] - y;
  const double dz = back[2] - z_down;   // forward() 也返回"向下为正"，同坐标系比较
  constexpr double kReachTolMm = 0.5;      // 数值残差约 1e-9，0.5mm 足够宽松又能挡住假解
  if (dx * dx + dy * dy + dz * dz > kReachTolMm * kReachTolMm) {
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    theta_deg[i] = th[i];
  }
  return true;
}

bool DeltaKinematics::forward(const double theta_deg[3], double xyz[3]) const
{
  // 由三个关节角求三个肘点，把动平台半径折入后得到三个虚拟球心，
  // 末端中心到各虚拟球心的距离恒为 L2，三球求交。
  double P[3][3];
  for (int i = 0; i < 3; ++i) {
    const double phi = i * 120.0 * kDeg2Rad;
    const double th = theta_deg[i] * kDeg2Rad;
    const double rr = (d1_ - d2_) + L1_ * std::cos(th);
    P[i][0] = rr * std::cos(phi);
    P[i][1] = rr * std::sin(phi);
    P[i][2] = -L1_ * std::sin(th);
  }

  // 球 1/2 分别减去球 0，得到两个线性方程 d·O = f, e·O = g
  const double dx = P[1][0] - P[0][0], dy = P[1][1] - P[0][1], dz = P[1][2] - P[0][2];
  const double ex = P[2][0] - P[0][0], ey = P[2][1] - P[0][1], ez = P[2][2] - P[0][2];

  const double f = 0.5 * (P[1][0] * P[1][0] + P[1][1] * P[1][1] + P[1][2] * P[1][2]
                          - P[0][0] * P[0][0] - P[0][1] * P[0][1] - P[0][2] * P[0][2]);
  const double g = 0.5 * (P[2][0] * P[2][0] + P[2][1] * P[2][1] + P[2][2] * P[2][2]
                          - P[0][0] * P[0][0] - P[0][1] * P[0][1] - P[0][2] * P[0][2]);

  const double D = dx * ey - dy * ex;
  if (std::fabs(D) < 1e-9) {
    return false;                                       // 三个肘点近共线，退化
  }

  // 解出 x、y 关于 z 的线性关系，代入球 0 化为一元二次方程
  const double xa = (f * ey - g * dy) / D;
  const double xb = (-dz * ey + ez * dy) / D;
  const double ya = (dx * g - ex * f) / D;
  const double yb = (-dx * ez + ex * dz) / D;

  const double Ax = xb, Bx = xa - P[0][0];
  const double Ay = yb, By = ya - P[0][1];
  const double A = Ax * Ax + Ay * Ay + 1.0;
  const double B = 2.0 * (Ax * Bx + Ay * By) - 2.0 * P[0][2];
  const double C = Bx * Bx + By * By + P[0][2] * P[0][2] - L2_ * L2_;

  const double disc = B * B - 4.0 * A * C;
  if (disc < 0.0) {
    return false;
  }

  const double sq = std::sqrt(disc);
  const double z1 = (-B + sq) / (2.0 * A);
  const double z2 = (-B - sq) / (2.0 * A);
  const double z = (z1 < z2) ? z1 : z2;                 // 取更靠下的工作位形

  xyz[0] = xa + xb * z;
  xyz[1] = ya + yb * z;
  xyz[2] = -z;                       // 对外 z 向下为正（内部 z 向上为正，故取反）
  return true;
}

}  // namespace delta_arm
