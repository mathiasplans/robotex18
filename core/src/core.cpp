#include "ros/ros.h"
#include "vision/Ball.h"
#include "vision/BasketRelative.h"
#include "serial/Ref.h"
#include "serial/WheelSpeed.h"
#include "core/Command.h"
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "debugcommands.hpp"
#include "wheelcontrol.hpp"
#include "statemachine.hpp"
#include "boost/bind.hpp"

#include "memory.hpp"
#include "speedcontrol.hpp"

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
 * Handle the message from serial package
 */
void referee_handler(const serial::Ref::ConstPtr& msg, StateMachine& sm){
  if(msg->start)
    sm.start_machine();
  else
    sm.stop_machine();
}

static Memory mem = Memory();

/**
 *
 */
void wheel_sum(const serial::WheelSpeed::ConstPtr& msg){
  mem << (wheel_speeds_t){msg->wheel1, msg->wheel2, msg->wheel3, msg->wheel4};
  // std::cout << "The sum of wheels: " <<  mem.get_orientation() << std::endl;
}

int main(int argc, char **argv){

  // Initialize ROS
  ros::init(argc, argv, "core");

  // Handle for the specific node
  ros::NodeHandle n;

  // Add the referee topic to topics pool
  ros::Publisher command_topic_out = n.advertise<core::Command>("commands", 50);

  // Create state machine instance
  StateMachine sm = StateMachine(command_topic_out, n, BLUE);

  // Subscribe to a message from vision
  ros::Subscriber image_processor = n.subscribe<vision::Ball>("ball", 50, boost::bind(vision_callback_ball, _1, boost::ref(sm)));

  ros::Subscriber basket_sub = n.subscribe<vision::Ball>("basket", 50, boost::bind(vision_callback_basket, _1, boost::ref(sm)));

  ros::Subscriber basket_depth = n.subscribe<vision::BasketRelative>("basketrelative", 50, boost::bind(basket_depth_callback, _1, boost::ref(sm)));

  // Subscribe to a message from serial
  ros::Subscriber referee_signal = n.subscribe<serial::Ref>("referee_signals", 50, boost::bind(referee_handler, _1, boost::ref(sm)));

  // Subscribe to Wheel Speeds 
  ros::Subscriber wheel_update = n.subscribe<serial::WheelSpeed>("wheelspeed", 50, wheel_sum);

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
