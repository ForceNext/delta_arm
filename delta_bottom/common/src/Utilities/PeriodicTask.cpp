/*!
 * @file PeriodicTask.cpp
 * @brief Implementation of a periodic function running in a separate thread.
 * Periodic tasks have a task manager, which measure how long they take to run.
 */
#ifdef linux
 #include <sys/timerfd.h>
#endif

#include <unistd.h>
#include <cmath>
#include <cstring>

#include "Utilities/PeriodicTask.h"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"


/*!
 * Construct a new task within a TaskManager
 * @param taskManager : Parent task manager
 * @param period : how often to run
 * @param name : name of task
 */
PeriodicTask::PeriodicTask(PeriodicTaskManager* taskManager, float period,
                           std::string name)
    : _period(period), _name(name) {
  taskManager->addTask(this);
}


void* PeriodicTask::func(void* p)
{
    PeriodicTask* task = (PeriodicTask*)p;
    task->_cpu_num = sysconf(_SC_NPROCESSORS_ONLN);
    printf_color(PrintColor::Blue,
                 "[CPU] System has %d processors \n",
                 task->_cpu_num);

    CPU_ZERO(&task->_mask);
    if(task->_cpu_id_set >= 0 && task->_cpu_id_set < task->_cpu_num)
    {
        CPU_SET(task->_cpu_id_set, &task->_mask);
        if(pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &task->_mask)!=0)
        {
          printf_color(PrintColor::Red,
                        "[CPU] Set thread affinity to CPU %d failed \n",
                        task->_cpu_id_set);
        }
        else
        {
          cpu_set_t cpuset;
          CPU_ZERO(&cpuset);
          if(pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)!=0 ||
             !CPU_ISSET(task->_cpu_id_set, &cpuset))
          {
            printf_color(PrintColor::Red,
                         "[CPU] Verify thread affinity to CPU %d failed \n",
                         task->_cpu_id_set);
          }
          else
          {
            printf_color(PrintColor::Green,
                         "[CPU] Set thread affinity to CPU %d success \n",
                         task->_cpu_id_set);
          }
        }
    }
    else
    {
        printf_color(PrintColor::Red,
                     "[CPU] Invalid CPU ID %d, not setting affinity \n",
                     task->_cpu_id_set);
    }

    Timer tim;

    int seconds = (int)task->_period;
    int nanoseconds = (int)(1e9 * std::fmod(task->_period, 1.f));
    unsigned long long missed = 0;
    auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec = seconds;
    timerSpec.it_value.tv_sec = seconds;
    timerSpec.it_value.tv_nsec = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;
    timerfd_settime(timerFd, 0, &timerSpec, nullptr);

    while (task->_running)
    {
        task->_lastPeriodTime = (float)tim.getSeconds();
        tim.start();

        task->run();//主任务函数

        task->_lastRuntime = (float)tim.getSeconds();
        task->_realRunTime = tim.getRealTime();

        int m = read(timerFd, &missed, sizeof(missed));//阻塞
        (void)m;
        task->_maxPeriod  = std::max(task->_maxPeriod, task->_lastPeriodTime);
        task->_maxRuntime = std::max(task->_maxRuntime, task->_lastRuntime);
    }

    return nullptr;
}


/*!
 * Begin running task
 */
void PeriodicTask::start() {
  if (_running) {
    printf("[PeriodicTask] Tried to start %s but it was already running!\n",
           _name.c_str());
    return;
  }
  init();
  int ret = 0;

  ret = pthread_attr_init(&_attr);
  if (ret != 0) {
    printf_color(PrintColor::Red,
                 "[PeriodicTask] %s: pthread_attr_init failed: %s\n",
                 _name.c_str(), strerror(ret));
    return;
  }

  ret = pthread_attr_setinheritsched(&_attr, PTHREAD_EXPLICIT_SCHED);
  if (ret != 0) {
    printf_color(PrintColor::Red,
                 "[PeriodicTask] %s: pthread_attr_setinheritsched failed: %s\n",
                 _name.c_str(), strerror(ret));
    return;
  }

  ret = pthread_attr_setschedpolicy(&_attr, SCHED_FIFO);
  if (ret != 0) {
    printf_color(PrintColor::Red,
                 "[PeriodicTask] %s: pthread_attr_setschedpolicy failed: %s\n",
                 _name.c_str(), strerror(ret));
    return;
  }

  ret = pthread_attr_setschedparam(&_attr, &_schedule_param);
  if (ret != 0) {
    printf_color(PrintColor::Red,
                 "[PeriodicTask] %s: pthread_attr_setschedparam failed: %s\n",
                 _name.c_str(), strerror(ret));
    return;
  }

  _running = true;

  ret = pthread_create(&_thread_id, &_attr, &func, this);
  if (ret != 0) {
    _running = false;
    printf_color(PrintColor::Red,
                 "[PeriodicTask] %s: pthread_create failed: %s\n",
                 _name.c_str(), strerror(ret));
    return;
  }
}

/*!
 * Stop running task
 */
void PeriodicTask::stop() {
  if (!_running) {
    printf("[PeriodicTask] Tried to stop %s but it wasn't running!\n",
           _name.c_str());
    return;
  }
  _running = false;
  printf("[PeriodicTask] Waiting for %s to stop...\n", _name.c_str());
  pthread_join(_thread_id, NULL);
  pthread_attr_destroy(&_attr);
  printf("[PeriodicTask] Done!\n");
  cleanup();
}

/*!
 * If max period is more than 30% over desired period, it is slow
 */
bool PeriodicTask::isSlow() {
  return _maxPeriod > _period * 1.3f || _maxRuntime > _period;
}

/*!
 * Reset max statistics
 */
void PeriodicTask::clearMax() {
  _maxPeriod = 0;
  _maxRuntime = 0;
}

/*!
 * Print the status of this task in the table format
 */
void PeriodicTask::printStatus() {
  if (!_running) return;
  if (isSlow()) {
    printf_color(PrintColor::Red, "|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n",
                 _name.c_str(), _lastRuntime, _maxRuntime, _period,
                 _lastPeriodTime, _maxPeriod);
  } else {
    printf("|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n", _name.c_str(),
           _lastRuntime, _maxRuntime, _period, _lastPeriodTime, _maxPeriod);
  }
}

PeriodicTaskManager::~PeriodicTaskManager() {}

/*!
 * Add a new task to a task manager
 */
void PeriodicTaskManager::addTask(PeriodicTask* task) {
  _tasks.push_back(task);
}

/*!
 * Print the status of all tasks and rest max statistics
 */
void PeriodicTaskManager::printStatus() {
  printf("\n----------------------------TASKS----------------------------\n");
  printf("|%-20s|%-6s|%-6s|%-6s|%-6s|%-6s\n", "name", "rt", "rt-max", "T-des",
         "T-act", "T-max");
  printf("-----------------------------------------------------------\n");
  for (auto& task : _tasks) {
    task->printStatus();
    task->clearMax();
  }
  printf("-------------------------------------------------------------\n\n");
}

/*!
 * Print only the slow tasks
 */
void PeriodicTaskManager::printStatusOfSlowTasks() {
  for (auto& task : _tasks) {
    if (task->isSlow()) {
      task->printStatus();
      task->clearMax();
    }
  }
}

/*!
 * Stop all tasks
 */
void PeriodicTaskManager::stopAll() {
  for (auto& task : _tasks) {
    task->stop();
  }
}
