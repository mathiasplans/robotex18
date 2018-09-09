cmake_minimum_required(VERSION 2.8.3)
project(begin)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy genmsg)

## Declare ROS messages and services
add_message_files(FILES Character.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sender src/begin.cpp)
target_link_libraries(sender ${catkin_LIBRARIES})
add_dependencies(sender begin_generate_messages_cpp)

add_executable(printer src/printer.cpp)
target_link_libraries(printer ${catkin_LIBRARIES})
add_dependencies(printer begin_generate_messages_cpp)
