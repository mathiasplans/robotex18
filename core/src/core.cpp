#include "ros/ros.h"
#include "vision/Ball.h"
#include "vision/BasketRelative.h"
#include "serial/Ref.h"
#include "core/Command.h"
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "debugcommands.hpp"
#include "wheelcontrol.hpp"
#include "statemachine.hpp"
#include "boost/bind.hpp"

#include <poll.h>
#include <regex>

/**
 * Handles the message from vision package
 */
void vision_callback_ball(const vision::Ball::ConstPtr& msg, StateMachine& sm){
  sm.update_ball_position(msg->ballX, msg->ballY, msg->width, msg->height);
}

void vision_callback_basket(const vision::Ball::ConstPtr& msg, StateMachine& sm){
  sm.update_basket_position(msg->ballX, msg->ballY, msg->width, msg->height);
}

void basket_depth_callback(const vision::BasketRelative::ConstPtr& msg, StateMachine& sm) {
  if(msg->depth > 10000){
    // sm.set_basket_dist(-1);
  }else{
    sm.set_basket_dist(msg->depth);
  }
}

/**
 * Handles the message from serial package
 */
void referee_handler(const serial::Ref::ConstPtr& msg, StateMachine& sm){
  if(msg->start)
    sm.start_machine();
  else
    sm.stop_machine();
}

int main(int argc, char **argv){

  // Initialize ROS
  ros::init(argc, argv, "core");

  // Handle for the specific node
  ros::NodeHandle n;

  // Add the referee topic to topics pool
  ros::Publisher command_topic_out = n.advertise<core::Command>("commands", 1000);

  // Create state machine instance
  StateMachine sm = StateMachine(command_topic_out, n);

  // Subscribe to a message from vision
  ros::Subscriber image_processor = n.subscribe<vision::Ball>("ball", 1000, boost::bind(vision_callback_ball, _1, boost::ref(sm)));

  ros::Subscriber basket_sub = n.subscribe<vision::Ball>("basket", 1000, boost::bind(vision_callback_basket, _1, boost::ref(sm)));

  ros::Subscriber basket_depth = n.subscribe<vision::BasketRelative>("basketrelative", 1000, boost::bind(basket_depth_callback, _1, boost::ref(sm)));

  // Subscribe to a message from serial
  ros::Subscriber referee_signal = n.subscribe<serial::Ref>("referee_signals", 1000, boost::bind(referee_handler, _1, boost::ref(sm)));

  std::cout << "Init finished" << std::endl;

  // // Get the 'ball' rolling. Get it? no
  // core::Bob command;
  // command.ball = s.searching_for_ball();
  // bob.publish(command);

  /* Polling for stdin */
  pollfd cinfd[1];
  cinfd[0].fd = fileno(stdin);
  cinfd[0].events = POLLIN;

  while(ros::ok()){
    handle_debug_command(sm, cinfd);

    // Run the State Machine once
    sm.state_machine();
    ros::spinOnce();
  }

  return 0;
}
