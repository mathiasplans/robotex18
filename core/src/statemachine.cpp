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
      counter++;
      //std::cout << "Looping\n";
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

void serial_init(int* serial){

  struct termios tty, tty_old;
  memset(&tty, 0, sizeof tty);

  /* Error Handling */
  if(tcgetattr(*serial, &tty) != 0)
    std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;

  /* Save old tty parameters */
  tty_old = tty;

  /* Set Baud Rate */
  cfsetospeed(&tty, (speed_t)B115200);
  cfsetispeed(&tty, (speed_t)B115200);

  /* Setting other Port Stuff */
  // Disable parity bit
  tty.c_cflag &= ~PARENB;

  // Default stop bits
  tty.c_cflag &= ~CSTOPB;

  // Serial byte is 8 bits
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  // No flow control
  tty.c_cflag &= ~CRTSCTS;

  // Non-blocking read
  tty.c_cc[VMIN] = 1;

  // Read timeout (0.5s)
  tty.c_cc[VTIME] = 5;

  // Enable Read and ignore control lines
  tty.c_cflag |= CREAD | CLOCAL;

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then apply attributes */
  tcflush(*serial, TCIFLUSH );

  if(tcsetattr(*serial, TCSANOW, &tty) != 0)
    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
}


StateMachine::StateMachine(void){

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
    write(serial, command.c_str(), command.size());
    std::cout << command << std::endl;
    usleep (1000000);
    return false;
  }
  // If then robot has found a ball, center in on it
  else{
    std::string command = stop();
    write(serial, command.c_str(), command.size());
    return true;
  }

  // Spin til green blob in middle of frame

}

bool StateMachine::center_on_ball(){
  // If the ball is not at the center of the frame
  if(object_position < POSITION_ERROR || object_position > POSITION_ERROR){
    std::string command = spin(SPIN_CENTER_SPEED * sgn(object_position));
    write(serial, command.c_str(), command.size());

    return false;
  }
  // If the ball is at the center of the frame
  else{
    std::string command = stop();
    write(serial, command.c_str(), command.size());
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
  std::cout << "Opening serial" << std::endl;

  serial = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

  if(serial == -1){
    std::cout << strerror(errno) << std::endl;
    return serial;
  }

  std::cout << "Serial opened" << std::endl;

  /* Serial communcication */
  /* https://stackoverflow.com/a/18134892 */
  serial_init(&serial);

  //state_thread.join();
  return 0;
}

void StateMachine::update_ball_position(int32_t x, int32_t width){
  object_position = width / 2 - x;
}

void StateMachine::set_object_in_sight(bool in_sight){
  object_in_sight = in_sight;
}
