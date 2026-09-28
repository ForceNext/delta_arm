# ---------------------------------------------------------------------------
# kinematics: 电机映射 / 扫描（MotorMapping）+ 正逆解（Kinematics）
# ---------------------------------------------------------------------------
add_library(kinematics STATIC
  src/MotorMapping.cpp
  src/Kinematics.cpp
)

target_include_directories(kinematics PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(kinematics PUBLIC
  drives
)
