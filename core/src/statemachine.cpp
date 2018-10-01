#include "defines.hpp"
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
      if(search_for_ball()) state = CENTER_ON_BALL;
      searching_ball = true;
      break;

    case CENTER_ON_BALL:
      if(center_on_ball()) state = MOVE_TO_BALL;
      searching_ball = true;
      break;

    case MOVE_TO_BALL:
      if(goto_ball()) state = SEARCH_BASKET;
      searching_ball = true;
      break;

    case SEARCH_BASKET:
      if(search_for_basket()) state = THROW;
      searching_ball = false;
      break;

    case THROW:
      if(throw_the_ball()) state = SEARCH_BALL;
      searching_ball = false;
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

StateMachine::StateMachine(ros::Publisher topic) : publisher(topic){

}

StateMachine::StateMachine(){

}

/**
 * Sign function
 * https://stackoverflow.com/a/4609795
 */
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

bool StateMachine::search_for_ball(){
  // If the robot hasn't found a ball yet
  if(!object_in_sight){
    std::string command = wheel::spin(SPIN_SEARCH_SPEED);
    serial_write(command);
    usleep(COMMAND_DELAY);
    return false;
  }

  // If then robot has found a ball, center in on it
  else{
    object_in_sight = false;
    std::string command = wheel::stop();
    serial_write(command);
    usleep(COMMAND_DELAY);
    return true;
  }
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
    std::string command = wheel::orbit(ORBIT_SPEED, BALL_IN_FRONT /* TODO: Replace with distance from ball instead */);
    serial_write(command);
    usleep(COMMAND_DELAY);
    return false;
  }

  // The basket and the ball are in the center of the frame
  else{
    std::string command = wheel::stop();
    serial_write(command);
    usleep(COMMAND_DELAY);
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

bool StateMachine::searching_for_ball(){
  return searching_ball;
}
