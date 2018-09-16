#include "ros/ros.h"
#include "core/Ball.h"

#include "statemachine.hpp"

/**
 * Handles
 */
void vision_callback(const core::Ball::ConstPtr& msg){
  ROS_INFO("I heard: [%d, %d]", msg->ballX, msg->width);
}


int main(){

  // Initialize ROS
  ros::init(argc, argv, "core");

  // Handle for the specific node
  ros::NodeHandle n;

  // Subscribe to a message from vision
  ros::Subscriber sub = n.subscribe("ball", 1000, vision_callback);

  // Initialize the CORE
  StateMachine();

  // Run ROS
  ros::spin();

  return 0;
}
