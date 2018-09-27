#include <cstdint>
#include <string>

/**
 *
 */
namespace wheel{

/**
 *
 */
typedef enum{
  WHEEL_1 = 0,    ///<
  WHEEL_2 = 120,  ///<
  WHEEL_3 = 240   ///<
}wheel_t;

/**
 *
 */
double speed_of_wheel(
  wheel_t wheel,      ///<
  int16_t speed,      ///<
  uint16_t direction  ///<
);

/**
 *
 */
std::string set_speed(
  wheel_t wheel,      ///<
  int16_t speed,      ///<
  uint16_t direction  ///<
);

/**
 *
 */
std::string move(
  int16_t speed,      ///<
  uint16_t direction  ///<
);

/**
 *
 */
std::string spin(
  int16_t ang_speed  ///<
);

/**
 *
 */
std::string circle(
  int16_t speed  ///<
);

/**
 *
 */
std::string stop();

/**
 *
 */
std::string thrower(
  uint16_t speed  ///<
);

/**
 *
 */
std::string thrower_stop();

};
