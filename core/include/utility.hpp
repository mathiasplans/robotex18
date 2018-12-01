//
// Created by Mr. Robot on 01-Dec-18.
//

#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

#include "xtensor/xfixed.hpp"
// Useful types
using move_vec_t     = xt::xtensor_fixed<double, xt::xshape<3,1> >;
using motor_speeds_t = xt::xtensor_fixed<double, xt::xshape<4,1> >;

constexpr double pi = 3.1415926535;

constexpr double to_rad(double deg) {
    return deg * pi / 180.0;
}

#endif //PROJECT_TYPES_H
