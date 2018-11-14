#pragma once
#include <cstdint>


/**
 *
 */
typedef struct{
  int16_t wheel1;
  int16_t wheel2;
  int16_t wheel3;
  int16_t wheel4;
}wheel_speeds_t;

/**
 *
 */
namespace speed{
  /**
   * This function takes the desired speeds and converts them to a smoothly accelerating speed over time
   * This function has internal state and if user wants to accelerate again after doing so, they
   * have to call restart_acceleration to reset the internal state.
   */
  wheel_speeds_t smooth_transition(
    wheel_speeds_t& wheel_speeds,  ///< Reference to the desired speed
    float ac_time                  ///< How many seconds does the acceleration take
  );

  /**
   * If the robot has finished it' accelerated move, this should be called
   * Not calling this has undefined behaviour, since the robot doesn't know
   * inherently if an action is over or not.
   */
  void restart_acceleration();

} /* namespace speed */
