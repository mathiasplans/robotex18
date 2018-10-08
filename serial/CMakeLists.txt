cmake_minimum_required(VERSION 2.8.3)
project(serial)


add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  genmsg
  message_generation
)

add_message_files(FILES Ref.msg WheelSpeed.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial src/serial.cpp)

add_dependencies(serial serial_generate_messages_cpp)

target_link_libraries(
  serial
  ${catkin_LIBRARIES}
)
