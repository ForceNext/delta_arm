cmake_minimum_required(VERSION 3.16)

project(delta_bottom LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)      # header-only, provides Eigen3::Eigen
find_package(Threads REQUIRED)     # std::thread in PeriodicTask

# ---------------------------------------------------------------------------
# yaml-cpp (vendored under third_party/)
# ---------------------------------------------------------------------------
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS  OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL      OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/yaml-cpp)

# ---------------------------------------------------------------------------
# common library (third_party/common)
# ---------------------------------------------------------------------------
add_library(common STATIC
  third_party/common/src/Utilities/PeriodicTask.cpp
  third_party/common/src/Utilities/utilities.cpp
  third_party/common/src/Utilities/Utilities_print.cpp
)
target_include_directories(common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/common/include
)
target_link_libraries(common PUBLIC
  Eigen3::Eigen
  Threads::Threads
)

# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
add_executable(delta_bottom user/main.cpp)
target_link_libraries(delta_bottom PRIVATE
  common
  yaml-cpp
)
