#include "ros/ros.h"
#include "vision/Ball.h"
#include "serial/Ref.h"
#include "core/Command.h"

#include "statemachine.hpp"

StateMachine sm;

/**
 * Handles the message from vision package
 */
void vision_callback(const vision::Ball::ConstPtr& msg){
  ROS_INFO("I heard: [%d, %d]", msg->ballX, msg->width);
  sm.update_ball_position(msg->ballX, msg->ballY, msg->width, msg->height);
  sm.set_object_in_sight(true);
}

/**
 * Handles the message from serial package
 */
void referee_handler(const serial::Ref::ConstPtr& msg){
  sm.set_stop_signal(!msg->start);
}

int main(int argc, char **argv){

  // Initialize ROS
  ros::init(argc, argv, "core");

  // Handle for the specific node
  ros::NodeHandle n;

  // Add the referee topic to topics pool
  ros::Publisher command_topic_out = n.advertise<core::Command>("commands", 1000);

  // Subscribe to a message from vision
  ros::Subscriber image_processor = n.subscribe("ball", 1000, vision_callback);

  // Subscribe to a message from serial
  ros::Subscriber referee_signal = n.subscribe("referee_signals", 1000, referee_handler);

  // Initialize the CORE
  if(sm.init() == -1){
    ros::shutdown();
    return 0;
  }

  std::cout << "Init finished\n";

  // Create state machine instance
  sm = StateMachine(command_topic_out);


  // Get the 'ball' rolling. Get it?
  core::Bob command;
  command.ball = s.searching_for_ball();
  bob.publish(command);

  while(ros::ok()){
    sm.state_machine();
    ros::spinOnce();
  }

  return 0;
}
