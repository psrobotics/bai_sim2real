cmake_minimum_required(VERSION 3.5)
project(bai_sim2real LANGUAGES CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Enable compiler warnings
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(Threads REQUIRED)
find_package(lcm REQUIRED)

include(${LCM_USE_FILE})

# Create executable
add_executable(imu_bridge 
               src/imu_bridge/imu_bridge.cpp
               src/imu_bridge/vofa_reader.cpp)
target_include_directories(imu_bridge 
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/imu_bridge)
target_link_libraries(imu_bridge PRIVATE 
                      Threads::Threads 
                      ${LCM_NAMESPACE}lcm)

