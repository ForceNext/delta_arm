#include "Kinematics.hpp"

#include "yaml-cpp/yaml.h"

#include <cmath>
#include <iostream>

namespace {
constexpr double PI      = 3.14159265358979323846;  // 圆周率
constexpr double DEG2RAD = PI / 180.0;               // 度 → 弧度
} // namespace

/**
 * @brief 构造 Kinematics，保存 delta 几何参数
 * @param d1 基座半径（mm，中心到肩关节轴）
 * @param L1 上臂长（mm，肩关节到肘关节）
 * @param L2 前臂/平行四边形杆长（mm，肘关节到动平台铰点）
 * @param d2 动平台半径（mm，中心到前臂铰点）
 * @param gear_ratio 三个主动臂传动比（电机角 = 关节角 × 传动比）
 */
Kinematics::Kinematics(double d1, double L1, double L2, double d2,
                       const double gear_ratio[3])
    : d1_(d1), L1_(L1), L2_(L2), d2_(d2),
      gear_ratio_{gear_ratio[0], gear_ratio[1], gear_ratio[2]},
      theta_{0.0, 0.0, 0.0}
{
}

/**
 * @brief 从 hardware_config.yaml 的 joint 段读取几何参数并构造
 * @param path 硬件配置文件路径
 * @return 读取成功返回按配置构造的 Kinematics；失败返回默认参数构造的实例
 * @note joint.jointlength 为 [d1, L1, L2, d2]，joint.gear_ratio 为 [g1, g2, g3]
 */
Kinematics Kinematics::fromYaml(const std::string& path)
{
    double d1 = 100.0, L1 = 150.0, L2 = 250.0, d2 = 50.0;  // 默认值（与 yaml 一致）
    double gr[3] = {2.0, 2.0, 2.0};
    try {
        YAML::Node root = YAML::LoadFile(path);
        auto jl = root["joint"]["jointlength"][0];  // [d1, L1, L2, d2]
        d1 = jl[0].as<double>();
        L1 = jl[1].as<double>();
        L2 = jl[2].as<double>();
        d2 = jl[3].as<double>();

        auto g = root["joint"]["gear_ratio"][0];    // [g1, g2, g3]
        gr[0] = g[0].as<double>();
        gr[1] = g[1].as<double>();
        gr[2] = g[2].as<double>();
    } catch (const std::exception& e) {
        std::cerr << "读取关节参数失败(" << e.what() << ")，使用默认值\n";
    }
    return Kinematics(d1, L1, L2, d2, gr);
}

/**
 * @brief 逆解：末端坐标 → 三个关节角（度），结果写入 theta_
 * @param x 末端 x 坐标（mm）
 * @param y 末端 y 坐标（mm）
 * @param z 末端 z 坐标（mm，正方向向上，平台在基座下方为负）
 * @return 目标点在工作空间内返回 true；越界（acos 参数超出 [-1,1]）返回 false
 * @note 约定 θ=0 上臂水平朝外、θ>0 上臂向下压；三臂方位角 0°/120°/240°。
 *       每个臂的两组解（肘向外/肘向内）取「肘向外下」的标准工作位形，
 *       若机器方向相反，把最后一行 acos 前的 - 换成 + 即可
 */
bool Kinematics::kinematics_ik(double x, double y, double z)
{
    for (int i = 0; i < 3; ++i) {
        double phi = i * 120.0 * DEG2RAD;   // 臂方位角 0°/120°/240°
        double c = std::cos(phi), s = std::sin(phi);

        // 目标点投影到第 i 臂竖直平面
        double Drad = x * c + y * s - (d1_ - d2_);   // 径向分量
        double Dtan = -x * s + y * c;                // 切向分量
        double M    = Drad * Drad + Dtan * Dtan + z * z + L1_ * L1_ - L2_ * L2_;
        double Rh   = std::sqrt(Drad * Drad + z * z);

        double val = M / (2.0 * L1_ * Rh);
        if (val < -1.0 || val > 1.0)
            return false;                            // 目标点超出工作空间

        // 两组解取「肘向外、上臂向下压」的标准工作位形；若机器方向相反，把 - 换成 + 即可
        theta_[i] = (std::atan2(-z, Drad) - std::acos(val)) / DEG2RAD;
    }
    return true;
}

/**
 * @brief 正解：三个关节角 theta_ → 末端坐标（mm）
 * @param xyz 出参：末端坐标 [x, y, z]（mm，z 正方向向上）
 * @return 成功返回 true 并写入 xyz；三球无交或退化（肘点近共线）返回 false
 * @note 由 theta_ 求三个肘点，折入动平台半径后得到三个虚拟球心，
 *       末端中心到虚拟球心的距离恒为 L2_，三球求交；取 z 较小的工作位形解
 */
bool Kinematics::kinematics_fk(double xyz[3]) const
{
    // 由三个关节角求三个肘点，把动平台半径折入后得到三个虚拟球心 E'_i，
    // 末端中心 O 到 E'_i 的距离恒为 L2_，三球求交。
    double P[3][3];
    for (int i = 0; i < 3; ++i) {
        double phi = i * 120.0 * DEG2RAD;
        double th  = theta_[i] * DEG2RAD;            // theta_ 单位是度
        double rr  = (d1_ - d2_) + L1_ * std::cos(th);
        P[i][0] = rr * std::cos(phi);
        P[i][1] = rr * std::sin(phi);
        P[i][2] = -L1_ * std::sin(th);
    }

    // 球 1/2 分别减去球 0，得到两个线性方程：d·O=f, e·O=g
    double dx = P[1][0] - P[0][0], dy = P[1][1] - P[0][1], dz = P[1][2] - P[0][2];
    double ex = P[2][0] - P[0][0], ey = P[2][1] - P[0][1], ez = P[2][2] - P[0][2];

    double f = 0.5 * (P[1][0]*P[1][0]+P[1][1]*P[1][1]+P[1][2]*P[1][2]
                    - P[0][0]*P[0][0]-P[0][1]*P[0][1]-P[0][2]*P[0][2]);
    double g = 0.5 * (P[2][0]*P[2][0]+P[2][1]*P[2][1]+P[2][2]*P[2][2]
                    - P[0][0]*P[0][0]-P[0][1]*P[0][1]-P[0][2]*P[0][2]);

    double D = dx * ey - dy * ex;
    if (std::fabs(D) < 1e-9)
        return false;   // 三个肘点近共线，退化

    // 解出 x、y 关于 z 的线性关系，代入球 0 化为一元二次方程
    double xa = (f * ey - g * dy) / D;
    double xb = (-dz * ey + ez * dy) / D;
    double ya = (dx * g - ex * f) / D;
    double yb = (-dx * ez + ex * dz) / D;

    double Ax = xb,          Bx = xa - P[0][0];
    double Ay = yb,          By = ya - P[0][1];
    double A  = Ax*Ax + Ay*Ay + 1.0;
    double B  = 2.0 * (Ax*Bx + Ay*By) - 2.0 * P[0][2];
    double C  = Bx*Bx + By*By + P[0][2]*P[0][2] - L2_*L2_;

    double disc = B*B - 4.0*A*C;
    if (disc < 0.0)
        return false;

    double sq = std::sqrt(disc);
    double z1 = (-B + sq) / (2.0 * A);
    double z2 = (-B - sq) / (2.0 * A);
    // 取更靠下（z 较小）的解，对应工作位形（z 正方向向上，平台在基座下方）
    double z = (z1 < z2) ? z1 : z2;

    xyz[0] = xa + xb * z;
    xyz[1] = ya + yb * z;
    xyz[2] = z;
    return true;
}
