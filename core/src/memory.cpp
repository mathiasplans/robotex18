#include "memory.hpp"

Memory::Memory(int tick_rate) : ticks_per_second(tick_rate) {

}

void Memory::operator<<(wheel_speeds_t wheel_speeds){
  // Determine the orientation
  orientation += (wheel_speeds.wheel1 + wheel_speeds.wheel2 + wheel_speeds.wheel3 + wheel_speeds.wheel4) / ticks_per_second;
}

float get_orientation(){
  return orientation;
}
