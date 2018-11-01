#include <cmath>
#include "defines.hpp"
#include "wheelcontrol.hpp"

#define SPEED_OF_WHEEL(wheel, sped, dir, ang) std::to_string((int16_t)speed_of_wheel(wheel, sped, dir, ang))

/**
 * Command format:
 * 'sd:wheel1:wheel2:wheel3:wheel4\r\n'
 */
#define PACKET1(sped, dir, ang) \
  std::string("sd:") + \
  SPEED_OF_WHEEL(WHEEL_1, sped, dir, ang) + \
  std::string(":") + \
  SPEED_OF_WHEEL(WHEEL_2, sped, dir, ang) + \
  std::string(":") + \
  SPEED_OF_WHEEL(WHEEL_3, sped, dir, ang) + \
  std::string(":") + \
  SPEED_OF_WHEEL(WHEEL_$, sped, dir, ang) + \
  std::string("\r\n")

#define PACKET2(sped1, sped2, sped3, sped4) \
  std::string("sd:") + \
  std::to_string(sped1) + \
  std::string(":") + \
  std::to_string(sped2) + \
  std::string(":") + \
  std::to_string(sped3) + \
  std::string(":") + \
  std::to_string(sped4) + \
  std::string("\r\n")

double wheel::speed_of_wheel(wheel_t wheel, double speed, double direction, double angular_velocity){
  return -MOVING_COEFFICIENT * (speed * cos((direction - wheel) * M_PI / 180) + angular_velocity * WHEEL_D);
}

std::string wheel::move(double speed, double direction, double angular_velocity){
  return PACKET1(speed, direction, angular_velocity);
}

std::string wheel::stop(){
  return PACKET2(0, 0, 0, 0);
}

std::string wheel::thrower(uint16_t speed){
  return std::string("d:") + std::to_string(speed) + std::string("\r\n");
}

std::string wheel::thrower_stop(){
  return std::string("d:0\r\n");
}

static uint16_t aim_position = 0;

std::string wheel::aim(uint16_t aim_power){
  aim_position += aim_power;
  return std::string("st:") + std::to_string(aim_power) + std::string("\r\n");
}

std::string wheel::deaim(){
  return std::string("st:-") + std::to_string(aim_position) + std::string("\r\n");
}
