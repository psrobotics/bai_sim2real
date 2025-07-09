# =================================================================
# Project Setup
# =================================================================

cmake_minimum_required(VERSION 3.5)
project(bai_sim2real LANGUAGES CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =================================================================
# Compiler Settings
# =================================================================

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =================================================================
# Dependencies
# =================================================================

# Rerun sdk for visualization
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

# System libraries
find_package(Threads REQUIRED)
find_package(lcm REQUIRED)

include(${LCM_USE_FILE})
include (GNUInstallDirs)

# =================================================================
# Executable Targets
# =================================================================

include_directories(
    ${PROJECT_SOURCE_DIR}/imu_bridge
    ${PROJECT_SOURCE_DIR}/mot_bridge
)

# --- IMU Bridge Executable ---
add_executable(
    imu_bridge
    ${SOURCE_DIR}/imu_bridge/imu_bridge.cpp
    ${SOURCE_DIR}/imu_bridge/vofa_reader.cpp
)
target_include_directories(
    imu_bridge
    PRIVATE ${SOURCE_DIR}/imu_bridge
)
target_link_libraries(
    imu_bridge PRIVATE
    Threads::Threads
    ${LCM_NAMESPACE}lcm
)

# --- Motor Bridge Executable ---
add_executable(
    mot_bridge
    ${SOURCE_DIR}/mot_bridge/MotorDriverTest.cpp
    ${SOURCE_DIR}/mot_bridge/MotorDriver.cpp
    ${SOURCE_DIR}/mot_bridge/CANInterface.cpp
    ${SOURCE_DIR}/mot_bridge/motor_group.cpp
)
target_include_directories(
    imu_bridge
    PRIVATE ${SOURCE_DIR}/mot_bridge
)
target_link_libraries(
    mot_bridge PRIVATE
    Threads::Threads
    rerun_sdk
    ${LCM_NAMESPACE}lcm
)
