#include "speedcontrol.hpp"
#include <ros/ros.h>

bool restart = false;
ros::Time now_time, start_time;

wheel_speeds_t speed::smooth_transition(wheel_speeds_t& wheel_speeds, float ac_time){
  // Calculate the acceleration of the robot
  wheel_speeds_t acceleration = {
    wheel_speeds.wheel1 / ac_time,
    wheel_speeds.wheel2 / ac_time,
    wheel_speeds.wheel3 / ac_time,
    wheel_speeds.wheel4 / ac_time
  };

  // If the robot has to restart it's acceleration (e.g. it decides to change it's trajectory)
  if(restart) start_time = ros::Time(0);

  // If the time of the start of the acceleration is unknown, create one
  if(start_time == ros::Time(0)) start_time = ros::Time::now();

  // Get the real time
  now_time = ros::Time::now();

  // Return the speed with no acceleration, since the ac_time is exceeded
  if(now_time - start_time >= ros::Duration(ac_time)) return wheel_speeds;

  // Retuen the speed with acceleration in mind
  return (wheel_speeds_t){
    acceleration.wheel1 * (int16_t)std::round((now_time - start_time).toSec() * 1000) / 1000,
    acceleration.wheel2 * (int16_t)std::round((now_time - start_time).toSec() * 1000) / 1000,
    acceleration.wheel3 * (int16_t)std::round((now_time - start_time).toSec() * 1000) / 1000,
    acceleration.wheel4 * (int16_t)std::round((now_time - start_time).toSec() * 1000) / 1000
  };
}
