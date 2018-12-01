cmake_minimum_required(VERSION 3.1)
project(core)

add_compile_options(-std=c++14 -g)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  vision
  genmsg
  serial
  cv_bridge
  geometry_msgs
  nav_msgs
)

find_package(Boost REQUIRED)

find_package(OpenCV 3 REQUIRED)

find_package(xtl REQUIRED)
find_package(xtensor REQUIRED)
find_package(xtensor-blas REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/
)

add_message_files(FILES Command.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs message_generation vision genmsg serial cv_bridge geometry_msgs nav_msgs
)

add_executable(core src/core.cpp src/wheelcontrol.cpp src/statemachine.cpp src/debugcommands.cpp include/statemachine.hpp include/wheelcontrol.hpp include/debugcommands.hpp src/speedcontrol.cpp include/speedcontrol.hpp src/memory.cpp include/memory.hpp)
add_dependencies(core ${catkin_EXPORTED_TARGETS})
add_dependencies(core core_generate_messages_cpp)


target_link_libraries(
  core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${XTENSOR_BLAS_TARGET}
  ${BLAS_LIBRARIES}
  ${LAPACK_LIBRARIES}
  xtensor
)
