cmake_minimum_required(VERSION 3.8)
project(Kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 用 C++17（ROS 2 Jazzy 默认即为 17，这里显式声明）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 逆解/正解 + 共享内存客户端，编成静态库，节点与（将来可能的）测试共用
add_library(kinematics_core STATIC
  src/Kinematics_ik.cpp
  src/shm_client.cpp
)
target_include_directories(kinematics_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# shm_open / mmap 在 glibc >= 2.34 已在 libc 内，无需显式 rt

# 节点：订阅 x,y,z → 逆解 → 写共享内存
add_executable(kinematics_node src/Kinematics_node.cpp)
target_link_libraries(kinematics_node
  kinematics_core
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

# 逆解自检程序（不接硬件）：ros2 run Kinematics ik_selftest
add_executable(ik_selftest src/ik_selftest.cpp)
target_link_libraries(ik_selftest kinematics_core)

install(TARGETS kinematics_node ik_selftest
  DESTINATION lib/${PROJECT_NAME}
)

# 手柄遥操作（Python 节点）：ros2 run Kinematics xbox_teleop
install(PROGRAMS scripts/xbox_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME xbox_teleop
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
