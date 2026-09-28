#include "arm_drive_task.hpp"

ArmDriveTask::ArmDriveTask(PeriodicTaskManager* taskManager, float period, std::string name)
    : PeriodicTask(taskManager, period, name)
{
    

}