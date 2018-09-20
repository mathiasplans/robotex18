cmake_minimum_required(VERSION 2.8.3)
project(core)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  vision
  serial
  message_generation
)

catkin_package(
)

ckage locations should be listed before other locations
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_executable(core src/core.cpp src/refereecontrol.cpp src/wheelcontrol.cpp src/statemachine.cpp include/statemachine.hpp include/wheelcontrol.hpp)
add_dependencies(core ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  core
  ${catkin_LIBRARIES}
)
