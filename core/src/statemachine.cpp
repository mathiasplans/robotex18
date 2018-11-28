#include "defines.hpp"
#include "statemachine.hpp"
#include "wheelcontrol.hpp"
#include "lookup_table.hpp"

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

/**
 * Sign function
 * https://stackoverflow.com/a/4609795
 */
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

void StateMachine::state_machine(void){
  static std::string command;
  static bool has_stopped = false;

  /* Main Loop */
  // If stop signale is set, then set to IDLE
  if(stop_signal || reset_signal){
    reset_signal = false;
    reset_internal_variables();
    if(!has_stopped){
      serial_write(wheel::stop());
      has_stopped = true;
    }
    set_state(IDLE);
    command_delay.sleep();
    return;
  }

  // If pause signal is set, do nothing
  if(pause_signal){
    if(!has_stopped){
      serial_write(wheel::stop());
      has_stopped = true;
    }
    command_delay.sleep();
    return;
  }

  has_stopped = false;
  
  /* State Handling */
  switch (state) {
    case TEST:
      // command = wheel::move(30, 90, 30, 0 ,0);
      // // command = wheel::move(30, 45, 0);
      // std::cout << command << std::endl;
      // serial_write(command);

      break;
    case IDLE:
      // Start searching for the ball
      ROS_INFO("Entering SEARCH_BALL state");
      set_state(SEARCH_BALL);

      break;

    case SEARCH_BALL:
      // If the robot hasn't found a ball yet
      if(ball_position_x < 0){
        // Circle around til a ball is found
        command = wheel::move(0, 0, SPIN_SEARCH_SPEED);
        serial_write(command);
      }

      // If then robot has found a ball, move to it
      else{
        // Stop the robot
        command = wheel::stop();
        serial_write(command);

        // Update the state
        ROS_INFO("Entering MOVE_TO_BALL state");
        set_state(MOVE_TO_BALL);
      }

      break;

    case MOVE_TO_BALL:
      // Check if the ball is still visible
      static int error_counter_mtb = 0;
      if(ball_position_x < 0 && error_counter_mtb++ > 1){
        ROS_INFO("Entering SEARCH_BALL state");
        set_state(SEARCH_BALL);
        break;
      }

      // Don't do anything, since using -1 in calculations is dangerous
      if(ball_position_x < 0) break;

      error_counter_mtb = 0;

      // If the ball is not in front of the robot
      if((ball_position_y <= BALL_IN_FRONT)){
        // In case the ball moves out of the pos error range then rotate a little bit
        double angv = 0;
        if(abs(ball_position_x - FRAME_WIDTH / 2) > POSITION_ERROR){
          angv += SPIN_SEARCH_SPEED / 1.4;
          if(ball_position_x > FRAME_WIDTH / 2){
            angv *= -1;
          }
        }

        command = wheel::move(MOVING_SPEED, 90, -(ball_position_x - FRAME_WIDTH / 2) * 0.01);
        serial_write(command);
      }

      // The ball is in front of the robot
      else{
        ROS_INFO("The ball is in front of the robot");
        command = wheel::stop();
        serial_write(command);
        ROS_INFO("Entering SEARCH_BASKET state");
        set_state(SEARCH_BASKET);
      }

      break;

    case SEARCH_BASKET:
      // Variable for holding the number of frames that the ball has been missing
      static int error_counter_sb = 0;
      // If the ball is not in sight anymore
      if(ball_position_x < 0 && error_counter_sb++ > 3){
        command = wheel::stop();
        serial_write(command);
        ROS_INFO("The ball was lost");
        ROS_INFO("Entering SEARCH_BALL state");
        set_state(SEARCH_BALL);
        break;
      };

      // Don't do anything, since using -1 in calculations is dangerous
      if(ball_position_x < 0) break;
      
      // Everything is fine after all
      error_counter_sb = 0;

      // If basket is sufficently in center
      if(abs(basket_position_x - FRAME_WIDTH / 2) < POSITION_ERROR && abs(ball_position_x - FRAME_WIDTH / 2) < POSITION_ERROR * 2) {
        command = wheel::stop();
        serial_write(command);
        ROS_INFO("Entering THROW state");
        set_state(THROW);
      }
       
      // Calculates the ball's y coordinate compensation speed and dir
      static int forwards_dir;
      forwards_dir = ball_position_y < BALL_IN_FRONT ? 90 : 270;

      static int forwards_speed;
      forwards_speed = abs(ball_position_y - BALL_IN_FRONT) * 0.5;

      static bool see_ball = false, seent_ball = false;

      if(abs(basket_position_x - FRAME_WIDTH / 2) < POSITION_ERROR * 2){
        std::cout << "Basket diff: " << basket_position_x - FRAME_WIDTH / 2 << "   ball diff " << ball_position_x - FRAME_WIDTH / 2 << std::endl;
        command = wheel::move(-ORBIT_SPEED * (basket_position_x - FRAME_WIDTH / 2) * 0.007 , 0, 0, 90, -(ball_position_x - FRAME_WIDTH / 2) * 0.015);
      }else
      // If basket is just in sight
      if(basket_position_x >= 0){
        // Added (int) conversion, not sure if correct. TODO
        // Calculate the orbiting speed, gets more slower the more the basket approaches the center. Minimum value is 15
        int sideways = std::max((int) abs((basket_position_x - FRAME_WIDTH / 2) * 0.15), 15);
        std::cout << sideways << std::endl;
        // Calculates the direction of the orbit.
        int orbit_dir = sgn(basket_position_x - FRAME_WIDTH / 2) == -1 ? 0 : 180;

        // Compiles the command for orbiting the ball
        command = wheel::move(sideways, orbit_dir, forwards_speed, forwards_dir, -(ball_position_x - FRAME_WIDTH / 2) * 0.01);
        if(!see_ball){
          ROS_INFO("I can see the basket");
          see_ball = true;
          seent_ball = false;
        }
      }

      // If basket is not in sight
      else{
        // Orbit aimlessly
        if(!seent_ball){
          ROS_INFO("I can't see the basket");
          seent_ball = true;
          see_ball = false;
        }
        ROS_INFO_STREAM("" << ball_position_x << ", " << -(ball_position_x - FRAME_WIDTH / 2) * 0.02);
        command = wheel::move(ORBIT_SPEED, 180, forwards_speed, forwards_dir,  -(ball_position_x - FRAME_WIDTH / 2) * 0.01);
        // serial_write(command);
      }

      serial_write(command);

      break;
    


    case THROW:
      // The ball is not thrown yet but we are getting close!
      if(substate[THROW] == THROW_AIM){
        // Configure the thrower
        configure_thrower(look_up(basket_dist));

        // Update the substate
        set_substate(THROW, THROW_GOAL);

      }else if(substate[THROW] == THROW_GOAL){
        configure_thrower(look_up(basket_dist));
        // Move towards the basket
        std::string command;
        if(false && ball_position_x >= 0){
          command = wheel::move(MOVING_SPEED_THROW * (ball_position_x - FRAME_WIDTH / 2) * 0.005 , 0, MOVING_SPEED_THROW, 90, -(basket_position_x - FRAME_WIDTH / 2) * 0.01);
        }else{
          command = wheel::move(MOVING_SPEED_THROW, 90, 0);
        }
        serial_write(command);

        // If ball is out of sight (very near the thrower)
        if(ball_position_x < 0){
          // Starts the thrower timer. At the end of this timer, the thrower
          // stops and the robot starts to search for a new ball
          throwing_timer.start();
          // set_state(TEST);
          // Update the substate
          set_substate(THROW, THROW_GOAL_NO_BALL);
        }

      }else if(substate[THROW] == THROW_GOAL_NO_BALL){
        // Move towards the basket
        std::string command = wheel::move(MOVING_SPEED_THROW, 90, 0);
        serial_write(command);

        // If the thrower timer has triggered
        if(throw_completed){
          // Reset the throw_complete state
          throw_completed = false;
          
          // Stop the thrower
          command = wheel::thrower(0);
          serial_write(command);
          
          // Stop the robot
          command = wheel::stop();
          serial_write(command);

          // Update the substate
          set_substate(THROW, THROW_AIM);
          ROS_INFO("Entering SEARCH_BALL state");
          set_state(SEARCH_BALL);
        }
      }

      break;

    default:
      /* Should never get here, ERROR! */
      break;

  }
}

void StateMachine::complete_throw(const ros::TimerEvent&){
  throw_completed = true;
  throwing_timer.stop();
  throwing_timer.setPeriod(ros::Duration(THROW_TIME));
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

void StateMachine::reset_substates(){
  substate[THROW] = THROW_AIM;
}

StateMachine::StateMachine(ros::Publisher& topic, ros::NodeHandle& node) : publisher(topic), command_delay(COMMAND_RATE), ros_node(node) {
  ROS_INFO("A StateMachine object was created with publisher");
  throwing_timer = ros_node.createTimer(ros::Duration(THROW_TIME), &StateMachine::complete_throw, this, true);
  throwing_timer.stop();
  throwing_timer.setPeriod(ros::Duration(THROW_TIME));
  reset_substates();
}

throw_info_t StateMachine::look_up(int distance){
  return getSpeedForDist(distance);
}

state_t StateMachine::get_state(){
  return state;
}

substate_t StateMachine::get_substate(state_t superstate){
  return substate[superstate];
}

void StateMachine::set_state(state_t superstate){
  state = superstate;
}

void StateMachine::set_substate(state_t superstate, substate_t new_substate){
  substate[superstate] = new_substate;
}

void StateMachine::update_ball_position(int x, int y, int width, int height){
  ball_position_x = x;
  ball_position_y = y;
}

void StateMachine::update_basket_position(int x, int y, int width, int height){
  basket_position_x = x + width/2;
  basket_position_y = y;
}

void StateMachine::set_basket_dist(int dist){
  basket_dist = dist;
}

void StateMachine::reset_internal_variables(){
  throw_completed = false;
}

void StateMachine::reset_machine(){
  reset_signal = true;
}

void StateMachine::stop_machine(){
  stop_signal = true;
}

void StateMachine::start_machine(){
  stop_signal = false;
  pause_signal = false;
}

void StateMachine::pause_machine(){
  pause_signal = true;
}

void StateMachine::set_throw_power(uint16_t power) {
  std::string command = wheel::thrower(power);
  serial_write(command);
}
void StateMachine::set_aimer_position(uint16_t angle) {
  std::string command = wheel::aim(angle);
  serial_write(command);
}

void StateMachine::configure_thrower(const throw_info_t& throw_parameters){
  set_throw_power(throw_parameters.dist);
  set_aimer_position(throw_parameters.angle);
}

void StateMachine::deaim(){
  serial_write(wheel::thrower_stop());
  serial_write(wheel::aim(1000));
}

// TODO: kui nurk on positiivne siis pööra paremale
