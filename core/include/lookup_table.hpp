#pragma once
#include <cstdint>

/**
 *
 */
typedef struct{
  uint16_t aim;
  uint16_t thrower;
}throw_parameters_t;

#define THROW_TYPE(aim, power) (throw_parameters_t){aim, power}

/**
 *
 */
const throw_parameters_t lookup_table[] = {
  // An example
  /* aim, thrower */
  THROW_TYPE(20, 1001),
  THROW_TYPE(19, 1010)
};
