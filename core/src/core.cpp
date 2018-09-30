#include "ros/ros.h"
#include "vision/Ball.h"
#include "core/Bob.h"

#include "statemachine.hpp"

StateMachine s;

// Handle for the specific node
ros::NodeHandle n;

// Publish Bob
ros::Publisher bob = n.advertise<core::Bob>("bob", 50);

/**
 * Handles
 */
void vision_callback(const vision::Ball::ConstPtr& msg){
  ROS_INFO("I heard: [%d, %d]", msg->ballX, msg->width);
  s.update_ball_position(msg->ballX, msg->ballY, msg->width, msg->height);
  s.set_object_in_sight(true);
  core::Bob command;
  command.ball = s.searching_for_ball();
  bob.publish(command);
}


int main(int argc, char **argv){

  // Initialize ROS
  ros::init(argc, argv, "core");

  // Subscribe to a message from vision
  ros::Subscriber sub = n.subscribe("ball", 1000, vision_callback);

  // Initialize the CORE
  if(s.init() == -1){
    ros::shutdown();
    return 0;
  }

  std::cout << "Init finished\n";
  // Run ROS
  ros::spinOnce();

  // Get the 'ball' rolling. Get it?
  core::Bob command;
  command.ball = s.searching_for_ball();
  bob.publish(command);

  while(ros::ok()){
    s.state_machine();
    ros::spinOnce();
    //std::cout << "hi\n";
  }

  std::cout << 1;

  return 0;
}
