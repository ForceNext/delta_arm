# ---------------------------------------------------------------------------
# drives: 串口驱动（SerialPort / ModbusMaster）静态库
# ---------------------------------------------------------------------------
add_library(drives STATIC
  src/serial_port.cpp
  src/modbus_master.cpp
)

target_include_directories(drives PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# yaml-cpp：serial_port.hpp 读取配置依赖 yaml-cpp（由顶层 CMakeLists.txt 定义）
target_link_libraries(drives PUBLIC
  yaml-cpp
)
