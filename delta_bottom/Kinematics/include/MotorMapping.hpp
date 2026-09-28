#ifndef MOTOR_MAPPING_HPP
#define MOTOR_MAPPING_HPP

class MotorMapping
{
private:

public:
    MotorMapping() = default;
    ~MotorMapping() = default;

    static double motorPosToAngle(int motorId, double pos);
    static double angleToMotorPos(int motorId, double angle);

    
};

#endif // MOTOR_MAPPING_HPP