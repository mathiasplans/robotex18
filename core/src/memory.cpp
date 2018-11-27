#include "memory.hpp"

Memory::Memory(){

}

void Memory::operator<<(wheel_speeds_t wheel_speeds){
  // Determine the orientation
  orientation += (wheel_speeds.wheel1 + wheel_speeds.wheel2 + wheel_speeds.wheel3 + wheel_speeds.wheel4);
}

float Memory::get_orientation(){
  return orientation;
}
