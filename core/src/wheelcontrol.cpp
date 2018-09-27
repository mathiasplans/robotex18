#include <cmath>
#include "wheelcontrol.hpp"

#define SPEED_OF_WHEEL(wheel, sped, dir) std::to_string((int16_t)speed_of_wheel(wheel, sped, dir))

/**
 * Command format:
 * 'sd:wheel1:wheel2:wheel3\r\n'
 */
#define PACKET1(sped, dir) \
  std::string("sd:") + \
  SPEED_OF_WHEEL(WHEEL_1, sped, dir) + \
  std::string(":") + \
  SPEED_OF_WHEEL(WHEEL_2, sped, dir) + \
  std::string(":") + \
  SPEED_OF_WHEEL(WHEEL_3, sped, dir) + \
  std::string("\r\n")

#define PACKET2(sped1, sped2, sped3) \
  std::string("sd:") + \
  std::to_string(sped1) + \
  std::string(":") + \
  std::to_string(sped2) + \
  std::string(":") + \
  std::to_string(sped3) + \
  std::string("\r\n")

double wheel::speed_of_wheel(wheel_t wheel, int16_t speed, uint16_t direction){
  return speed * cos((direction - wheel) * M_PI / 180);
}

std::string wheel::move(int16_t speed, uint16_t direction){
  return PACKET1(speed, direction);
}

std::string wheel::spin(int16_t ang_speed){
  return PACKET2(ang_speed, ang_speed, ang_speed);
}

std::string wheel::circle(int16_t speed){
  return PACKET2(speed, 0, 0);
}

std::string wheel::stop(){
  return PACKET2(0, 0, 0);
}

std::string wheel::thrower(uint16_t speed){
  return std::string("d:") + std::to_string(speed) + std::string("\r\n");
}

std::string wheel::thrower_stop(){
  return std::string("d:0\r\n");
}
