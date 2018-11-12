#include "speedcontrol.cpp"
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
  if(restart) start_time = 0;

  // If the time of the start of the acceleration is unknown, create one
  if(start_time == 0) start_time = ros::Time::now();

  // Get the real time
  new_time = ros::Time::now();

  // Return the speed with no acceleration, since the ac_time is exceeded
  if(new_time - start_time >= ac_time) return wheel_speeds;

  // Retuen the speed with acceleration in mind
  return (wheel_speeds_t){
    acceleration.wheel1 * (new_time - start_time),
    acceleration.wheel2 * (new_time - start_time),
    acceleration.wheel3 * (new_time - start_time),
    acceleration.wheel4 * (new_time - start_time)
  };
}
