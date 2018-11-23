#pragma once

typedef struct{
  float x;
  float y;
}point_t;

class Memory{
private:
  float orientation;    ///< Angle of the robot. 0 is the pink baset.
  int tick_per_second;  ///< How many times is the position updates

public:
  /**
   * Main constructor
   */
  Memory(
      int tick_rate  ///<
  );

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
}
