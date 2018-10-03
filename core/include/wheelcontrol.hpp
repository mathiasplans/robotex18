#include <cstdint>
#include <string>

/**
 * Namespace for all functions related to motion of a robot
 */
namespace wheel{

  /**
   * Enum of wheel angles
   */
  typedef enum{
    WHEEL_1 = 0,    ///< Angle of the first wheel
    WHEEL_2 = 120,  ///< Angle of the second wheel
    WHEEL_3 = 240   ///< Angle of the third wheel
  }wheel_t;

  /**
   * Get the speed of the specific wheel with motion meant for specific speed and direction
   */
  double speed_of_wheel(
    wheel_t wheel,      ///< [in] Index of the wheel
    double speed,      ///< [in] Speed of the movement
    double direction,  ///< [in] Direction of movement
    double angular_velocity
  );

  /**
   * Get the command for moving a single wheel
   */
  std::string set_speed(
    wheel_t wheel,      ///< [in] Index of the wheel
    int16_t speed       ///< [in] Speed of the movement
  );

  /**
   * Get the command for moving in a specific direction and speed
   */
  std::string move(
    double speed,      ///< [in] Speed of the movement
    double direction,  ///< [in] Direction of the movement
    double angular_velocity
  );

  /**
   * Get the command for spinning with specific speed
   */
  // std::string spin(
  //   int16_t ang_speed  ///< [in] Speed of the spinning motion
  // );

  /**
   * Get the command for stopping the motion
   */
  std::string stop();

  /**
   * Get the command for setting the speef of the thrower
   */
  std::string thrower(
    uint16_t speed  ///< [in] Speed of the thrower. Ranges from 1001 to 2000.
  );

  /**
   * Get the command for stopping the motion of the thrower
   */
  std::string thrower_stop();

  /**
   * Get the command for orbiting an object with specific speed and distance
   */
  // std::string orbit(
  //   int16_t speed,   ///< [in] Speed of the orbit
  //   uint16_t radius  ///< [in] Distance from the object that is orbited. In cm.
  // );

}; /* namespace wheel */
