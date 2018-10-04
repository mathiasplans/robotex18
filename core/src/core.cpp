#include "ros/ros.h"
#include "vision/Ball.h"
#include "core/Bob.h"
#include "boost/bind.hpp"

#include "statemachine.hpp"

StateMachine s;



/**
 * Handles
 */
void vision_callback(const vision::Ball::ConstPtr& msg, ros::Publisher& bob){
  // ROS_INFO("I heard: [%d, %d]", msg->ballX, msg->ballY);
  
  if(msg->ballX < 0) {
    if(s.get_state() != THROW) s.reset_machine();
  }else{
    s.update_ball_position(msg->ballX, msg->ballY, msg->width, msg->height);
    s.set_object_in_sight(true);
  }
  
  core::Bob command;
  command.ball = s.searching_for_ball();
  bob.publish(command);
}

/**
 * Handles
 * Note: Ball message is reused for basket aswell
 */
void vision_callback2(const vision::Ball::ConstPtr& msg, ros::Publisher& bob){
  // ROS_INFO("I heard: [%d, %d]", msg->ballX, msg->ballY);
  
  if(msg->ballX < 0) {
    // s.reset_machine();
  }else{
    s.update_basket_position(msg->ballX, msg->ballY, msg->width, msg->height);
    s.set_basket_in_sight(true);
  }
  core::Bob command;
  command.ball = s.searching_for_ball();
  bob.publish(command);
}

int main(int argc, char **argv){

  // Initialize ROS
  ros::init(argc, argv, "core");

  // Handle for the specific node
  ros::NodeHandle n;

  // Publish Bob
  ros::Publisher bob = n.advertise<core::Bob>("bob", 50);

  // Subscribe to a message from vision
  ros::Subscriber ball_sub = n.subscribe<vision::Ball>("ball", 1000, boost::bind(vision_callback, _1, bob));

  // Subscribe to a message from vision
  ros::Subscriber basket_sub = n.subscribe<vision::Ball>("basket", 1000, boost::bind(vision_callback2, _1, bob));

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
