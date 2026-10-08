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
    bool kinematics_fk(double xyz[3]) const;
    double theta(int i) const { return theta_[i]; }
    // 电机角（度）：关节角 × 传动比
    double motorTheta(int i) const { return theta_[i] * gear_ratio_[i]; }

private:
    double d1_, L1_, L2_, d2_;
    double gear_ratio_[3];
    double theta_[3];
};

#endif
