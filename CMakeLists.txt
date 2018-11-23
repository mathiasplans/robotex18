cmake_minimum_required(VERSION 3.12)
project(src)

set(CMAKE_CXX_STANDARD 14)

include_directories(core/include)
include_directories(vision/include)

add_executable(src
        core/include/debugcommands.hpp
        core/include/defines.hpp
        core/include/lookup_table.hpp
        core/include/speedcontrol.hpp
        core/include/statemachine.hpp
        core/include/wheelcontrol.hpp
        core/src/core.cpp
        core/src/debugcommands.cpp
        core/src/speedcontrol.cpp
        core/src/statemachine.cpp
        core/src/wheelcontrol.cpp
        serial/src/serial.cpp
        vision/include/ball_detection.hpp
        vision/src/ball_detection.cpp
        vision/src/basket_detection.cpp
        vision/src/camsim.cpp
        vision/src/realsense.cpp)
