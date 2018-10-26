#include <cstdint>

/**
 * 
 */
typedef struct{
  uint16_t aim;
  uint16_t thrower;
}throw_parameters_t;

/**
 *
 */
throw_parameters_t lookup_table[] = {
  // This does not work due to it being non-trivial designator
  [25] = (throw_parameters_t){20, 30}
};
