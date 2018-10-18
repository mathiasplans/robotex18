#include "defines.hpp"
#include "statemachine.hpp"
#include "wheelcontrol.hpp"

#include <cstdio>
#include <cerrno>
#include <termios.h>
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
  if(stop_signal /*|| reset_signal*/){
    reset_signal = false;
    serial_write(wheel::stop());
    state = IDLE;
    command_delay.sleep();
    return;
  }

  // std::cout << state << std::endl;

  /* State Handling */
  switch (state) {
    case IDLE:
      // Start searching for the ball
      state = SEARCH_BALL;

      break;
    case SEARCH_BALL:
      if(search_for_ball()) state = MOVE_TO_BALL;
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
    std::cout << "here0\n";
      break;

    default:
      /* Should never get here, ERROR! */
      break;
  }
}

void StateMachine::serial_write(std::string string){
  // Create a message
  core::Command msg;
  msg.command = string;

  // Publish the message
  publisher.publish(msg);

  // Sleep for the duration of COMMAND_DELAY
  command_delay.sleep();
}

void StateMachine::set_stop_signal(bool ref_signal){
  stop_signal = ref_signal;
}

StateMachine::StateMachine(ros::Publisher& topic) : publisher(topic), command_delay(COMMAND_RATE) {
  std::cout << "A StateMachine object was created with publisher" << std::endl;
}

StateMachine::StateMachine() : command_delay(COMMAND_RATE) {
  std::cout << "A StateMachine object was created without publisher" << std::endl;
}

/**
 * Sign function
 * https://stackoverflow.com/a/4609795
 */
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

bool StateMachine::search_for_ball(){
  // std::cout << "Search for ball: "  << object_in_sight << std::endl;
  // If the robot hasn't found a ball yet
  if(!object_in_sight){
    std::string command = wheel::move(0, 0, SPIN_SEARCH_SPEED);
    serial_write(command);
    // std::cout << "hi\n";
    return false;
  }

  // If then robot has found a ball, center in on it
  else{
    // object_in_sight = false;
    std::string command = wheel::stop();
    serial_write(command);
    return true;
  }
}

bool StateMachine::center_on_ball(){
  // If the ball is not at the center of the frame
  if(abs(object_position_x - FRAME_WIDTH / 2) > POSITION_ERROR){
    double angv = SPIN_CENTER_SPEED;
    if(object_position_x > FRAME_WIDTH / 2){
      angv *= -1;
    }

    std::string command = wheel::move(0, 0, angv);
    serial_write(command);
    return false;
  }

  // If the ball is at the center of the frame
  else{
    std::string command = wheel::stop();
    serial_write(command);
    return true;
  }
}


bool StateMachine::goto_ball(){
  // If the ball is not in front of the robot
  if((object_position_y < BALL_IN_FRONT)){
    // In case the ball moves out of the pos error range then rotate a little bit
    double angv = 0;
    if(abs(object_position_x - FRAME_WIDTH / 2) > POSITION_ERROR){
      angv += SPIN_SEARCH_SPEED / 1.4;
      if(object_position_x > FRAME_WIDTH / 2){
        angv *= -1;
      }
    }
    
    std::string command = wheel::move(MOVING_SPEED, 90, angv);
    serial_write(command);

    return false;
  }

  // The ball is in front of the robot
  else{
    std::string command = wheel::stop();
    serial_write(command);
    return true;
  }
}

bool StateMachine::search_for_basket(){
  // The basket and the ball are not in the center of the frame
  std::string command;
  bool ret = false;
  std::cout << basket_state << std::endl;

  if(basket_state == ORBIT_BALL){
    if(abs(basket_position_x - FRAME_WIDTH / 2) < POSITION_ERROR * 5 ) {
      // command = wheel::stop();
      // ret = true;
      // basket_state = ORBIT_BALL;
      basket_state = CENTER_BASKET;
    }


    command = wheel::move(ORBIT_SPEED, 180, -(object_position_x - FRAME_WIDTH / 2) * 0.03);

  }else if(basket_state == CENTER_BASKET){
    if(abs(basket_position_x - FRAME_WIDTH / 2) < POSITION_ERROR) basket_state = ORBIT_BASKET;

    double angv = SPIN_CENTER_SPEED * 1.4;
    if(basket_position_x > FRAME_WIDTH / 2){
      angv *= -1;
    }

    command = wheel::move(0, 0, angv);

  }else if(basket_state == ORBIT_BASKET){
    int16_t sign = 1;
    if(basket_position_x > FRAME_WIDTH / 2){
      sign *= -1;
    }

    int16_t dir = 0;
    if((object_position_x - FRAME_WIDTH / 2) < 0) dir = 180;

    command = wheel::move(ORBIT_SPEED * 0.7, dir, -(basket_position_x - FRAME_WIDTH / 2) * 0.02);

    if(abs(object_position_x - FRAME_WIDTH / 2) < POSITION_ERROR){
      command = wheel::stop();
      ret = true;
      basket_state = ORBIT_BALL;
    }
  }

  serial_write(command);
  return ret;
}

bool StateMachine::throw_the_ball(){
  // The ball is not thrown yet but we are getting close!
  if(true /* Placeholder */){
    // Thrower motor control
    std::string command = wheel::thrower(THROWER_SPEED);
    serial_write(command);

    // Wheel control
    command = wheel::move(MOVING_SPEED_THROW, 90, 0);
    serial_write(command);
    return false;
  }

  // The ball has been thrown
  else{
    // Thrower motor control
    std::string command = wheel::thrower_stop();
    serial_write(command);

    // Wheel control
    command = wheel::stop();
    serial_write(command);
  }
}


state_t StateMachine::get_state(){
  return state;
}


void StateMachine::update_ball_position(int16_t x, int16_t y, uint16_t width, uint16_t height){
  object_position_x = x;
  object_position_y = y;
}

void StateMachine::update_basket_position(int16_t x, int16_t y, uint16_t width, uint16_t height){
  basket_position_x = x + width/2;
  basket_position_y = y;
}

void StateMachine::set_object_in_sight(bool in_sight){
  object_in_sight = in_sight;
  // std::cout << "Function called\n";
  // std::cout << "Obj in sight: " << object_in_sight << std::endl;
}

void StateMachine::set_basket_in_sight(bool in_sight){
  basket_in_sight = in_sight;
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
