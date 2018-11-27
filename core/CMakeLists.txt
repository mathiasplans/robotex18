cmake_minimum_required(VERSION 2.8.3)
project(core)
add_compile_options(-std=c++14 -g)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  vision
  genmsg
  serial
)

find_package(Boost REQUIRED)

include_directories(
  include/
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_message_files(FILES Command.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

add_executable(core src/core.cpp src/wheelcontrol.cpp src/statemachine.cpp src/debugcommands.cpp include/statemachine.hpp include/wheelcontrol.hpp include/debugcommands.hpp src/speedcontrol.cpp include/speedcontrol.hpp src/memory.cpp include/memory.hpp)
add_dependencies(core ${catkin_EXPORTED_TARGETS})
add_dependencies(core core_generate_messages_cpp)

target_link_libraries(
  core
  ${catkin_LIBRARIES}
)
