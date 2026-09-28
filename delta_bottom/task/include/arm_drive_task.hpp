#ifndef ARM_DRIVE_TASK_HPP
#define ARM_DRIVE_TASK_HPP

#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"
#include "MotorMapping.hpp"
#include "yaml-cpp/yaml.h"

class ArmDriveTask : public PeriodicTask
{
public:
    ArmDriveTask(PeriodicTaskManager* taskManager, float period, std::string name);
    ~ArmDriveTask() override = default;     
    void init() override;
    void run() override;
    void cleanup() override;

private:

};

#endif