#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

class Kinematics
{
private:
    unsigned int L1 = 0;
    unsigned int L2 = 0;
    unsigned int d1 = 0;
    unsigned int d2 = 0;
    float theta[3] = {0.0, 0.0, 0.0}; //三个电机角度
public:
    Kinematics(/* args */);
    ~Kinematics(); 
    void kinematics_ik();  //逆解算
    void kinematics_fk();  //正解算 
};

#endif