# ---------------------------------------------------------------------------
# drives: 串口驱动（SerialPort / SerialTask）静态库
# ---------------------------------------------------------------------------
add_library(drives STATIC
  src/serial_port.cpp
  src/serial_task.cpp
)

target_include_directories(drives PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# - yaml-cpp 由顶层 CMakeLists.txt 通过 add_subdirectory(third_party/yaml-cpp) 定义
# - common：serial_task.hpp 依赖 Utilities/Timer.h、PeriodicTask.h（位于
#   third_party/common/include，通过 common 的 PUBLIC 头文件目录传递）
# - Threads：serial_task.cpp 使用 std::thread / pthread 系列函数
target_link_libraries(drives PUBLIC
  common
  yaml-cpp
  Threads::Threads
)
