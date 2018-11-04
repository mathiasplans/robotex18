#include <cmath>
#include "defines.hpp"
#include "wheelcontrol.hpp"

#define SPEED_OF_WHEEL(wheel, sped, dir, ang) std::to_string((int16_t)speed_of_wheel(wheel, sped, dir, ang))

double wheel::speed_of_wheel(wheel_t wheel, double speed, double direction, double angular_velocity){
  return -MOVING_COEFFICIENT * (speed * cos((direction - wheel) * M_PI / 180) + angular_velocity * WHEEL_D);
}

std::string wheel::move(double speed, double direction, double angular_velocity){
  return std::string("sd:") +
    SPEED_OF_WHEEL(WHEEL_1, speed, direction, angular_velocity) +
    std::string(":") +
    SPEED_OF_WHEEL(WHEEL_2, speed, direction, angular_velocity) +
    std::string(":") +
    SPEED_OF_WHEEL(WHEEL_3, speed, direction, angular_velocity) +
    std::string(":") +
    SPEED_OF_WHEEL(WHEEL_4, speed, direction, angular_velocity) +
    std::string("\r\n");
}

std::string wheel::stop(){
  return std::string("sd:0:0:0:0\r\n");
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
  return std::string("a:") + std::to_string(aim_power) + std::string("\r\n");
}

std::string wheel::deaim(){
  aim_position = 0;
  return std::string("a:-") + std::to_string(aim_position) + std::string("\r\n");
}
