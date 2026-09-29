#ifndef ARM_DRIVE_TASK_HPP
#define ARM_DRIVE_TASK_HPP

#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"
#include "MotorMapping.hpp"
#include "yaml-cpp/yaml.h"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"

class ArmDriveTask : public PeriodicTask
{
public:
    ArmDriveTask(PeriodicTaskManager* taskManager, float period, std::string name);
    ~ArmDriveTask() override = default;     
    void init() override;
    void run() override;
    void cleanup() override;
    void print_loop_rate();
private:
    Timer time_ctrl;
    uint16_t loop_counter_;
    double time_start_sec_;
    float dt;
};

#endif