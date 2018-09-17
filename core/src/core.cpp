#include "ros/ros.h"
#include "vision/Ball.h"

#include "statemachine.hpp"

StateMachine s;

/**
 * Handles
 */
void vision_callback(const vision::Ball::ConstPtr& msg){
  ROS_INFO("I heard: [%d, %d]", msg->ballX, msg->width);
  s.update_ball_position(msg->ballX, msg->width);
  s.set_object_in_sight(true);
}


int main(int argc, char **argv){

  // Initialize ROS
  ros::init(argc, argv, "core");

  // Handle for the specific node
  ros::NodeHandle n;

  // Subscribe to a message from vision
  // ros::Subscriber sub = n.subscribe("ball", 1000, vision_callback);

  // Initialize the CORE
  

  if(s.init() == -1){
    ros::shutdown();
    return 0;
  }

  std::cout << "Init finished\n";
  // Run ROS
  ros::spinOnce();

  while(ros::ok()){
    s.state_machine();
    //std::cout << "hi\n";
  }

  std::cout << 1;

  return 0;
}
