#include <cstdint>
#include <string>
#include <array>
#include "xtensor/xfixed.hpp"
#include "utility.hpp"

/**
 * Namespace for all functions related to motion of a robot
 */


namespace wheel{

  /**
   * Enum of wheel angles
   */
  typedef enum{
    WHEEL_1 = 45,  ///< Angle of the first wheel
    WHEEL_2 = 135,  ///< Angle of the second wheel
    WHEEL_3 = 225,   ///< Angle of the third wheel
    WHEEL_4 = 315,  ///< Angle of the fourth wheel
  }wheel_t;

  /**
   * Get the speed of the specific wheel with motion meant for specific speed and direction
   */
  double speed_of_wheel(
    wheel_t wheel,           ///< Index of the wheel
    double speed,            ///< Speed of the movement
    double direction,        ///< Direction of movement
    double angular_velocity  ///< Spinning speed while moving
  );

  /**
   * Get the command for moving a single wheel
   */
  std::string set_speed(
    wheel_t wheel,      ///< Index of the wheel
    int16_t speed       ///< Speed of the movement
  );

  /**
   * Get the command for moving in a specific direction and speed
   */
  std::string move(
    double speed,            ///< Speed of the movement
    double direction,        ///< Direction of the movement
    double angular_velocity  ///< Spinning speed while moving
  );

  /**
   * Get the command for moving with two polar coordinates which get added together
   */
  std::string move(
    double speed1,            ///< Speed of the movement of the first vector
    double direction1,        ///< Direction of the movement of the first vector
    double speed2,            ///< Speed of the movement of the second vector
    double direction2,        ///< Direction of the movement of the second vector
    double angular_velocity  ///< Spinning speed while moving
  );

  /**
   * Get the command for stopping the motion
   */
  std::string stop();

    /*
   * Convert euclidean speeds in m/s to motor speeds.
   */
  motor_speeds_t to_motor(move_vec_t euclidean_speeds);


    /*
   * Convert motor speeds to a string
   */
  std::string to_speed_str(motor_speeds_t motor_speeds);


    /**
   * Get the command for setting the speef of the thrower
   */
  std::string thrower(
    uint16_t speed  ///< Speed of the thrower. Ranges from 1001 to 2000.
  );

  /**
   * Get the command for stopping the motion of the thrower
   */
  std::string thrower_stop();

  /**
   * This function returns a command for aiming down sight
   */
  std::string aim(
    uint16_t aim_power
  );
}; /* namespace wheel */
