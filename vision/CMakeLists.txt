cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  genmsg
  message_generation
)

find_package(OpenCV 3 REQUIRED)
add_message_files(FILES Ball.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(camsim src/camsim.cpp)

target_link_libraries(camsim
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(ball_detection src/ball_detection.cpp)

target_link_libraries(ball_detection
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)


