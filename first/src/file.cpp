#include "ros/ros.h"
#include <fstream>
#include <string>

using namespace ros;

int main(int argc, char **argv){
  init(argc, argv, "file");
  NodeHandle n;
  Rate loopRate(2);

  while(ok()){
    std::ifstream infile;
    infile.open("/home/robot/test");
    
    std::string s;
    std::getline(infile, s);

    ROS_INFO("read from file: %s", s.c_str());

    loopRate.sleep();
    

  }
  return 0;
}
