# Builds vacuum_core + the JNI glue into libvacuum_jni.so for the app.
# vacuum_core lives at the repo root and is shared with the host test build.

cmake_minimum_required(VERSION 3.22)
project(vacuum_jni LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Repo-relative: app/src/main/cpp -> ../../../../../../vacuum_core
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../vacuum_core
  ${CMAKE_CURRENT_BINARY_DIR}/vacuum_core
)

add_library(vacuum_jni SHARED jni_bridge.cpp)
target_link_libraries(vacuum_jni PRIVATE vacuum_core)
find_library(log-lib log)
target_link_libraries(vacuum_jni PRIVATE ${log-lib})
