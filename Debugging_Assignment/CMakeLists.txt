cmake_minimum_required(VERSION 3.14)
project(rover_safety_monitor CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(rover_safety_monitor
    main.cpp
    safety_monitor.cpp
)

target_compile_options(rover_safety_monitor PRIVATE -Wall -Wextra -Wpedantic)
