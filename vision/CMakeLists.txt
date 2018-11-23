cmake_minimum_required(VERSION 2.8.3)
project(vision)

add_compile_options(-std=c++14 -g)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  genmsg
  message_generation
  realsense2
)

find_package(Boost REQUIRED)

find_package(OpenCV 3 REQUIRED)
add_message_files(FILES Ball.msg)
add_message_files(FILES BasketRelative.msg)
add_message_files(FILES Threshold.msg)
add_message_files(FILES Break.msg)
add_message_files(FILES Output_type.msg)
add_service_files(FILES Objinf.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
  include/
)

add_executable(camsim src/camsim.cpp)
add_dependencies(camsim vision_generate_messages_cpp)

target_link_libraries(camsim
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(rs src/realsense.cpp)
add_dependencies(rs vision_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(rs
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(ball_detection src/ball_detection.cpp include/ball_detection.hpp)
add_dependencies(ball_detection vision_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(ball_detection
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(basket_detection src/basket_detection.cpp include/ball_detection.hpp)
add_dependencies(basket_detection vision_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(basket_detection
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
