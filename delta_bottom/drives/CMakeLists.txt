# ---------------------------------------------------------------------------
# drives: 串口驱动（SerialPort）静态库
# ---------------------------------------------------------------------------
add_library(drives STATIC
  src/serial_port.cpp
)

target_include_directories(drives PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# yaml-cpp 由顶层 CMakeLists.txt 通过 add_subdirectory(third_party/yaml-cpp) 定义
target_link_libraries(drives PUBLIC
  yaml-cpp
)
