#pragma once
#include "speedcontrol.hpp"

typedef struct{
  float x;
  float y;
}point_t;

class Memory{
private:
  float orientation = 0;    ///< Angle of the robot. 0 is the pink baset.

public:
  /**
   * Main constructor
   */
  Memory();

  /**
   * Operator overload for <<.
   * Feed the wheelspeeds to this class
   * by using << operator.
   */
  void operator<<(
      wheel_speeds_t wheel_speed
  );

  /**
   * Returns the orinetation of the robot
   */
  float get_orientation();
};
