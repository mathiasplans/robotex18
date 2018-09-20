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

void StateMachine::state_machine(void){
  /* Main Loop */

  // If stop signale is set, then set to IDLE
  if(stop_signal){
    state = IDLE;
    return;
  }

  /* State Handling */
  switch (state) {
    case IDLE:
      // Start Searching Ball
      //usleep(1000);
      state = SEARCH_BALL;

      break;
    case SEARCH_BALL:
      if(search_for_ball(/* Timeout perhaps */)) state = MOVE_TO_BALL;
      break;
    case SEARCH_BASKET:
      if(search_for_basket(/* Timeout perhaps */)) state = THROW;
      break;
    case MOVE_TO_BALL:
      if(goto_ball(/* Timeout perhaps */)) state = SEARCH_BASKET;
      break;
    case THROW:
      if(throw_the_ball(/* Timeout perhaps */)) state = SEARCH_BALL;
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
}

void StateMachine::set_stop_signal(bool ref_signal){
  stop_signal = ref_signal;
}

StateMachine::StateMachine(ros::Publisher topic) : publisher(topic){

}

StateMachine::StateMachine(){

}

#define SPIN_SEARCH_SPEED 10
#define SPIN_CENTER_SPEED 5
#define MOVING_SPEED      5
#define POSITION_ERROR    5

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
    std::string command = move(SPIN_SEARCH_SPEED, 0);
    //std::cout << "Hello\n";
    serial_write(command);
    std::cout << command << std::endl;
    usleep (1000000);
    return false;
  }
  // If then robot has found a ball, center in on it
  else{
    std::string command = stop();
    serial_write(command);
    return true;
  }

  // Spin til green blob in middle of frame

}

bool StateMachine::center_on_ball(){
  // If the ball is not at the center of the frame
  if(object_position < POSITION_ERROR || object_position > POSITION_ERROR){
    std::string command = spin(SPIN_CENTER_SPEED * sgn(object_position));
    serial_write(command);

    return false;
  }
  // If the ball is at the center of the frame
  else{
    std::string command = stop();
    serial_write(command);
    return true;
  }
}

bool StateMachine::search_for_basket(){

  // std::string command = move(MOVING_SPEED, 0 /* Go Staright */);

  // Spin til basket in middle of frame
  return false;

}

bool StateMachine::goto_ball(){

  // Simply move to the ball
  return false;

}

bool StateMachine::throw_the_ball(){

  // Move forward and consome the ball
  return false;

}

int StateMachine::init(){
    return 0;
}

void StateMachine::update_ball_position(int32_t x, int32_t width){
  object_position = width / 2 - x;
}

void StateMachine::set_object_in_sight(bool in_sight){
  object_in_sight = in_sight;
}
