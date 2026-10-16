#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <string>

class Kinematics
{
public:
    Kinematics(double d1, double L1, double L2, double d2, const double gear_ratio[3]);
    ~Kinematics() = default;
    static Kinematics fromYaml(const std::string& path);
    bool kinematics_ik(double x, double y, double z);
    bool kinematics_fk(double xyz[3]) const;                             // 用当前 theta_（命令值）
    bool kinematics_fk(const double joint_theta[3], double xyz[3]) const; // 用给定关节角（实测值）
    double theta(int i) const { return theta_[i]; }          // 关节角（度）
    double gearRatio(int i) const { return gear_ratio_[i]; } // 第 i 臂减速比
    // 下发方向：电机角（度）= 关节角 × 减速比
    double motorTheta(int i) const { return theta_[i] * gear_ratio_[i]; }
    // 回读方向：关节角（度）= 电机角 ÷ 减速比（读回实测电机位置做正解时使用）
    double jointThetaFromMotor(int i, double motor_theta) const
    {
        return motor_theta / gear_ratio_[i];
    }

private:
    bool fkFromAngles(const double joint_theta[3], double xyz[3]) const;  // 正解公共实现
    double d1_, L1_, L2_, d2_;
    double gear_ratio_[3];
    double theta_[3];
};

#endif
