#include "ros/ros.h"
#include "first/Text.h"
#include <sstream>

using namespace ros;

int main(int argc, char **argv){
  init(argc, argv, "talker");
  NodeHandle n;
  Publisher chatter_pub = n.advertise<first::Text>("chatter", 1000);

  Rate loopRate(10);

  int count = 0;
  while(ok()){
    first::Text msg;

    std::stringstream ss;
    ss << "Hey yall nr " << count;

    msg.c = ss.str();

    ROS_INFO("%s", msg.c.c_str());

    chatter_pub.publish(msg);

    spinOnce();

    loopRate.sleep();
    count++;

  }
    
  return 0;
}
