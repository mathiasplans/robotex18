#include <cmath>
#include "defines.hpp"
#include "wheelcontrol.hpp"

#define SPEED_OF_WHEEL(wheel, sped, dir, ang) std::to_string((int16_t)speed_of_wheel(wheel, sped, dir, ang))

/**
 * Command format:
 * 'sd:wheel1:wheel2:wheel3\r\n'
 */
#define PACKET1(sped, dir, ang) \
  std::string("sd:") + \
  SPEED_OF_WHEEL(WHEEL_1, sped, dir, ang) + \
  std::string(":") + \
  SPEED_OF_WHEEL(WHEEL_2, sped, dir, ang) + \
  std::string(":") + \
  SPEED_OF_WHEEL(WHEEL_3, sped, dir, ang) + \
  std::string("\r\n")

#define PACKET2(sped1, sped2, sped3) \
  std::string("sd:") + \
  std::to_string(sped1) + \
  std::string(":") + \
  std::to_string(sped2) + \
  std::string(":") + \
  std::to_string(sped3) + \
  std::string("\r\n")

double wheel::speed_of_wheel(wheel_t wheel, double speed, double direction, double angular_velocity){
  return (speed * cos((direction - wheel) * M_PI / 180) + angular_velocity * WHEEL_D);
}

// For moving and turning at the same time

std::string wheel::move(double speed, double direction, double angular_velocity){
  return PACKET1(speed, direction, angular_velocity);
}

// For moving straight

std::string wheel::move(double speed, double direction){
  return PACKET1(speed, direction, 0);
}

// std::string wheel::spin(int16_t ang_speed){
//   return PACKET2(ang_speed, ang_speed, ang_speed);
// }

std::string wheel::stop(){
  return PACKET2(0, 0, 0);
}

std::string wheel::thrower(uint16_t speed){
  return std::string("d:") + std::to_string(speed) + std::string("\r\n");
}

std::string wheel::thrower_stop(){
  return std::string("d:0\r\n");
}

// std::string wheel::orbit(int16_t speed, uint16_t radius){
//   uint16_t spinning_speed = WHEEL_R * speed / radius;
//   return PACKET2(
//     (speed_of_wheel(WHEEL_1, speed, (M_PI / 2)) + spinning_speed),
//     (speed_of_wheel(WHEEL_2, speed, (M_PI / 2)) + spinning_speed),
//     (speed_of_wheel(WHEEL_3, speed, (M_PI / 2)) + spinning_speed)
//   );
// }
