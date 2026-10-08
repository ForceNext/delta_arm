# ---------------------------------------------------------------------------
# task: 机械臂驱动任务（arm_drive_task）
# ---------------------------------------------------------------------------
add_library(task STATIC
  src/arm_drive_task.cpp
)

target_include_directories(task PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# arm_drive_task 依赖电机映射/正逆解（kinematics）与共享内存（common）；
# drives（ModbusMaster）经 kinematics 传递而来。
target_link_libraries(task PUBLIC
  kinematics
  common
)
