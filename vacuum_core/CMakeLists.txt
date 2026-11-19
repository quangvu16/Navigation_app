# vacuum_core — portable C++17 robotics core (host tests + Android NDK).
#
# Host:   cmake -B build -DVACUUM_BUILD_TESTS=ON && cmake --build build && ctest
# Android: the app's CMakeLists (android/VacuumNav/app/src/main/cpp) adds this
# directory with VACUUM_BUILD_TESTS=OFF — no GoogleTest on-device.

cmake_minimum_required(VERSION 3.16)
project(vacuum_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(VACUUM_BUILD_TESTS "Build GoogleTest unit tests (host only)" ON)

add_library(vacuum_core STATIC
  sim/sim_world.cpp
  sim/sim_robot.cpp
)

target_include_directories(vacuum_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}          # for sim/sim_world.h includes
)

if(NOT ANDROID)
  target_compile_options(vacuum_core PRIVATE -Wall -Wextra)
endif()

if(VACUUM_BUILD_TESTS AND NOT ANDROID)
  enable_testing()
  add_subdirectory(third_party/googletest)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  add_executable(vacuum_core_tests
    tests/test_math.cpp
    tests/test_config.cpp
    tests/test_sim_world.cpp
    tests/test_sim_robot.cpp
    tests/test_scan_transform.cpp
  )
  target_link_libraries(vacuum_core_tests PRIVATE vacuum_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(vacuum_core_tests)
endif()
