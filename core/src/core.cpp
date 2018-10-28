#include "ros/ros.h"
#include "vision/Ball.h"
#include "serial/Ref.h"
#include "core/Command.h"
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "statemachine.hpp"
#include "boost/bind.hpp"

#include <regex>

/**
 * Handles the message from vision package
 */
void vision_callback(const vision::Ball::ConstPtr& msg, StateMachine& sm){
  if(msg->ballX < 0) {
    if(sm.get_state() != THROW) sm.reset_machine();
  }else{
    // std::cout << "Found a ball!" << std::endl;
    sm.update_ball_position(msg->ballX, msg->ballY, msg->width, msg->height);
    sm.set_object_in_sight(true);
  }
}

void vision_callback2(const vision::Ball::ConstPtr& msg, StateMachine& sm){
  // ROS_INFO("I heard: [%d, %d]", msg->ballX, msg->ballY);

  if(msg->ballX < 0) {
    // s.reset_machine();
  }else{
    sm.update_basket_position(msg->ballX, msg->ballY, msg->width, msg->height);
    sm.set_basket_in_sight(true);
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

// Mimicking pythons split function
std::vector<std::string> split(std::string str, std::string sep = " ") {
  std::vector<std::string> result;

  size_t start = 0;
  size_t end   = str.find(sep);
  while(end != std::string::npos) {
    result.push_back(str.substr(start, end-start));
    start = end + sep.length();
    end = str.find(sep, start);
  }
  result.push_back(str.substr(start, end));
  return result;
}

/**
 *
 */
void handle_debug_command(StateMachine& sm){
  static std::string input_command;

  // Handle the input commands
  if(std::getline(std::cin, input_command)){
    if(input_command == std::string("start")){
      sm.start_machine();
      std::cout << "The robot has been started" << std::endl;
    }
    else if(input_command == std::string("stop")){
      sm.stop_machine();
      std::cout << "The robot has been  stopped" << std::endl;
    }
    else if(input_command == std::string("reset")){
      sm.reset_machine();
      std::cout << "The robot has been reset" << std::endl;
    }
    else if(input_command == std::string("pause")){
      sm.pause_machine();
      std::cout << "The robot has been paused" << std::endl;
    }
    // Currently doesn't work
    else if(std::regex_search(input_command, std::regex("set state [A-Z_]+"))){
      std::string state_string = split(input_command)[2];

      std::cout << "Switching to state: " << state_string << std::endl;
      if(state_string == "IDLE") sm.set_state(IDLE);
      else if(state_string == "SEARCH_BALL") sm.set_state(SEARCH_BALL);
      else if(state_string == "CENTER_ON_BALL") sm.set_state(CENTER_ON_BALL);
      else if(state_string == "MOVE_TO_BALL") sm.set_state(MOVE_TO_BALL);
      else if(state_string == "SEARCH_BASKET") sm.set_state(SEARCH_BASKET);
      else if(state_string == "THROW") sm.set_state(THROW);
      else std::cout << "Entered state is invalid" << std::endl;
    }
    // Currently doesn't work
    else if(std::regex_search(input_command, std::regex("set substate [A-Z_]+"))){
      std::string substate_string = split(input_command)[2];

      std::cout << "Swithcing to substate: " << substate_string << std::endl;
      if(substate_string == "BASKET_ORBIT_BALL") sm.set_substate(SEARCH_BALL, BASKET_ORBIT_BALL);
      else if(substate_string == "BASKET_CENTER_BASKET") sm.set_substate(SEARCH_BASKET, BASKET_CENTER_BASKET);
      else if(substate_string == "BASKET_ORBIT_BASKET") sm.set_substate(SEARCH_BASKET, BASKET_ORBIT_BASKET);
      else if(substate_string == "THROW_AIM") sm.set_substate(THROW, THROW_AIM);
      else if(substate_string == "THROW_GOAL") sm.set_substate(THROW, THROW_GOAL);
      else if(substate_string == "THROW_GOAL_NO_BALL") sm.set_substate(THROW, THROW_GOAL_NO_BALL);
      else if(substate_string == "THROW_DEAIM") sm.set_substate(THROW, THROW_DEAIM);
      else std::cout << "Entered substate is invalid" << std::endl;
    }
    else if(input_command == "reset substates"){
      sm.reset_substates();
      std::cout << "The substates have been set to their default value" << std::endl;
    }
    else if(input_command == "get state") std::cout << "The State Machine is in " << std::to_string(sm.get_state()) << " state" << std::endl;
    else if(std::regex_search(input_command, std::regex("set thrower \\d+"))){
      std::string thrower_string = split(input_command)[2];
      sm.set_throw_power(std::stoi(thrower_power));
      std::cout << "Set the thrower to: " << thrower_string << std::endl;
    }
    else if(std::regex_search(input_command, std::regex("set aimer \\d+"))){
      std::string aimer_string = split(input_command)[2];
      sm.set_aimer_position(std::stoi(aimer_string));
      std::cout << "Set the aimer to: " << aimer_string << std::endl;
    }
    else std::cout << "Entered command is invalid" << std::endl;

    // Clear the string for the new commands to be read
    input_command.clear();
  }
}

int main(int argc, char **argv){

  // Initialize ROS
  ros::init(argc, argv, "core");

  // Handle for the specific node
  ros::NodeHandle n;

  // Add the referee topic to topics pool
  ros::Publisher command_topic_out = n.advertise<core::Command>("commands", 1000);

  // Create state machine instance
  StateMachine sm = StateMachine(command_topic_out);

  // Subscribe to a message from vision
  ros::Subscriber image_processor = n.subscribe<vision::Ball>("ball", 1000, boost::bind(vision_callback, _1, boost::ref(sm)));

  ros::Subscriber basket_sub = n.subscribe<vision::Ball>("basket", 1000, boost::bind(vision_callback2, _1, boost::ref(sm)));

  // Subscribe to a message from serial
  ros::Subscriber referee_signal = n.subscribe<serial::Ref>("referee_signals", 1000, boost::bind(referee_handler, _1, boost::ref(sm)));

  std::cout << "Init finished" << std::endl;

  // // Get the 'ball' rolling. Get it?
  // core::Bob command;
  // command.ball = s.searching_for_ball();
  // bob.publish(command);

  while(ros::ok()){
    handle_debug_command(sm);

    // Run the State Machine once
    sm.state_machine();
    ros::spinOnce();
  }

  return 0;
}
