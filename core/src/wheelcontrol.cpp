#include <cmath>
#include "defines.hpp"
#include "wheelcontrol.hpp"
#include <iostream>

#include <opencv2/core/core.hpp>

using namespace cv;

double coup_data[4][3] = {{-3984.15795879, -3984.15886237,  5634.45085889},
                          {-3984.15976596,  3984.1570552 ,  5634.45085889},
                          { 3984.15615161,  3984.16066955,  5634.45085889},
                          { 3984.16157313, -3984.15524803,  5634.45085889}};

Mat coup = Mat(4, 3, CV_64F, coup_data);

std::array<int, 4> wheel::to_motor(double x, double y, double phi) {
  // Turn the arguments into a matrix
  double euclidean_speeds_data[3][1] = {{x}, {y}, {phi}};
  Mat euclidean_speeds = Mat(3,1, CV_64F, euclidean_speeds_data);

  Mat motor_speeds = coup * euclidean_speeds;

  // Round and arrayify the speeds.
  return { (int) std::round(motor_speeds.at<double>(0,0)),
           (int) std::round(motor_speeds.at<double>(0,1)),
           (int) std::round(motor_speeds.at<double>(0,2)),
           (int) std::round(motor_speeds.at<double>(0,3))};

}

std::string wheel::to_speed_str(std::array<int, 4> motor_speeds) {
  return "sd:" + std::to_string(motor_speeds[0]) + ":"
               + std::to_string(motor_speeds[1]) + ":"
               + std::to_string(motor_speeds[2]) + ":"
               + std::to_string(motor_speeds[3]);
}


#define SPEED_OF_WHEEL(wheel, sped, dir, ang) std::to_string((int16_t)speed_of_wheel(wheel, sped, dir, ang))

double wheel::speed_of_wheel(wheel_t wheel, double speed, double direction, double angular_velocity){
  return -MOVING_COEFFICIENT * (speed * cos(direction - wheel * M_PI / 180) + angular_velocity * WHEEL_D);
}

std::string wheel::move(double speed, double direction, double angular_velocity){
  direction *= M_PI / 180;
  return std::string("sd:") +
    SPEED_OF_WHEEL(WHEEL_1, speed, direction, angular_velocity) +
    std::string(":") +
    SPEED_OF_WHEEL(WHEEL_2, speed, direction, angular_velocity) +
    std::string(":") +
    SPEED_OF_WHEEL(WHEEL_3, speed, direction, angular_velocity) +
    std::string(":") +
    SPEED_OF_WHEEL(WHEEL_4, speed, direction, angular_velocity);
}

std::string wheel::move(double speed1, double direction1, double speed2, double direction2,  double angular_velocity){
  direction1 *= M_PI / 180;
  direction2 *= M_PI / 180;

  double speed = sqrt(pow(speed1, 2) + pow(speed2, 2) + 2 * speed1 * speed2 * cos(direction2 - direction1));
  double direction = direction1 + atan2(speed2 * sin(direction2 - direction1), speed1 + speed2 * cos(direction2 - direction1));
  
  // std::cout << "Speed: " << speed << " direction: " << direction * 180 / M_PI << std::endl;

  return std::string("sd:") +
    SPEED_OF_WHEEL(WHEEL_1, speed, direction, angular_velocity) +
    std::string(":") +
    SPEED_OF_WHEEL(WHEEL_2, speed, direction, angular_velocity) +
    std::string(":") +
    SPEED_OF_WHEEL(WHEEL_3, speed, direction, angular_velocity) +
    std::string(":") +
    SPEED_OF_WHEEL(WHEEL_4, speed, direction, angular_velocity);
}

std::string wheel::stop(){
  return std::string("sd:0:0:0:0");
}

std::string wheel::thrower(uint16_t speed){
  return std::string("d:") + std::to_string(speed);
}

std::string wheel::thrower_stop(){
  return std::string("d:0");
}

std::string wheel::aim(uint16_t aim_power){
  return std::string("a:") + std::to_string(aim_power);
}
