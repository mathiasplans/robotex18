#include "statemachine.hpp"
#include "wheelcontrol.hpp"

#include <cstdio>      // standard input / output functions
#include <cerrno>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <cstring>
#include <iostream>
#include <fcntl.h>

#include <cstdlib>
#include <unistd.h>

#include <core/Command.h>
#include <cmath>

void StateMachine::state_machine(void){
  /* Main Loop */
  
  // If stop signale is set, then set to IDLE
  if(stop_signal || reset_signal){
    reset_signal = false;
    state = IDLE;
    return;
  }

  /* State Handling */
  switch (state) {
    case IDLE:
      // Start searching for the ball
      state = SEARCH_BALL;
      
      break;
    case SEARCH_BALL:
      if(search_for_ball()) state = SEARCH_BALL;
      break;

    case CENTER_ON_BALL:
      if(center_on_ball()) state = MOVE_TO_BALL;
      break;

    case MOVE_TO_BALL:
      if(goto_ball()) state = SEARCH_BASKET;
      break;

    case SEARCH_BASKET:
      if(search_for_basket()) state = THROW;
      break;

    case THROW:
      if(throw_the_ball()) state = SEARCH_BALL;
      break;

    case CORRECT_POSITION:
      break;

    default:
      /* Should never get here, ERROR! */
      break;
  }
}

void StateMachine::serial_write(std::string string){
  core::Command msg;
  msg.command = string;
  publisher.publish(msg);
  usleep(1000);
}

void StateMachine::set_stop_signal(bool ref_signal){
  stop_signal = ref_signal;
}

StateMachine::StateMachine(ros::Publisher& topic){
  std::cout << "ho";
  publisher = topic;
  
}

StateMachine::StateMachine(){

}

#define SPIN_SEARCH_SPEED     10
#define SPIN_CENTER_SPEED     1
#define MOVING_SPEED          5
#define MOVING_SPEED_THROW    1
#define POSITION_ERROR        25
#define CAMERA_FOV_X          M_PI * 2 / 9 /* 40 degrees */
#define BALL_IN_FRONT         /* Height at which the ball is in front of the robot */
#define COMMAND_RATE          30 /* Commands per second */
#define COMMAND_DELAY         1000000 / COMMAND_RATE
#define THROWER_SPEED         1500

/**
 * Sign function
 * https://stackoverflow.com/a/4609795
 */
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

bool StateMachine::search_for_ball(){
  // If the robot hasn't found a ball yet
  //if(!object_in_sight){
    // std::string command = wheel::spin(SPIN_SEARCH_SPEED);
    // NOTE: Testing
    std::string command = std::string("sd:5:1:1\r\n");
    serial_write(command);
    usleep(COMMAND_DELAY);
    return false;
 // }

  // If then robot has found a ball, center in on it
  //else{
    //object_in_sight = false;
    //std::string command = wheel::stop();
    //serial_write(command);
    //usleep(COMMAND_DELAY);
  //  return true;
 // }
}

bool StateMachine::center_on_ball(){
  // If the ball is not at the center of the frame
  if(object_position_x < -POSITION_ERROR || object_position_x > POSITION_ERROR){
    std::string command = wheel::spin(SPIN_CENTER_SPEED * sgn(object_position_x));
    serial_write(command);
    usleep(COMMAND_DELAY);
    return false;
  }

  // If the ball is at the center of the frame
  else{
    std::string command = wheel::stop();
    serial_write(command);
    usleep(COMMAND_DELAY);
    return true;
  }
}


bool StateMachine::goto_ball(){
  // If the ball is not in front of the robot
  if(object_position_y > BALL_IN_FRONT + POSITION_ERROR || object_position_y < BALL_IN_FRONT - POSITION_ERROR){
    std::string command = wheel::move(MOVING_SPEED, object_position_x * CAMERA_FOV_X);
    serial_write(command);
    usleep(COMMAND_DELAY);
    return false;
  }

  // The ball is in front of the robot
  else{
    std::string command = wheel::stop();
    serial_write(command);
    usleep(COMMAND_DELAY);
    return true;
  }
}

bool StateMachine::search_for_basket(){
  // The basket and the ball are not in the center of the frame
  if(object_position_x < -POSITION_ERROR || object_position_x > POSITION_ERROR){

    return false;
  }

  // The basket and the ball are in the center of the frame
  else{

    return true;
  }

}

bool StateMachine::throw_the_ball(){
  // The ball is not thrown yet but we are getting close!
  if(true /* Placeholder */){
    // Thrower motor control
    std::string command = wheel::thrower(THROWER_SPEED);
    serial_write(command);
    usleep(COMMAND_DELAY / 2);

    // Wheel control
    command = wheel::move(MOVING_SPEED, 0);
    serial_write(command);
    usleep(COMMAND_DELAY / 2);
    return false;
  }

  // The ball has been thrown
  else{
    // Thrower motor control
    std::string command = wheel::thrower_stop();
    serial_write(command);
    usleep(COMMAND_DELAY / 2);

    // Wheel control
    command = wheel::stop();
    serial_write(command);
    usleep(COMMAND_DELAY / 2);
  }
}

int StateMachine::init(){
    return 0;
}

void StateMachine::update_ball_position(int16_t x, int16_t y, uint16_t width, uint16_t height){
  object_position_x = -width / 2 + x;
  object_position_y = height / y;
}

void StateMachine::set_object_in_sight(bool in_sight){
  object_in_sight = in_sight;
}

void StateMachine::reset_machine(){
  reset_signal = true;
}

void StateMachine::stop_machine(){
  stop_signal = true;
}

void StateMachine::start_machine(){
  stop_signal = false;
}
