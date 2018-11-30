#include <ros/ros.h>
#include <ros/console.h>
#include <serial/Ref.h>
#include <serial/WheelSpeed.h>
#include <core/Command.h>
#include <std_msgs/String.h>

#include <sstream>
#include <string>

#include <cstdio>
#include <cerrno>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <fcntl.h>

#include <cstdlib>
#include <unistd.h>

/**
 * Referee commands
 */

std::string field_id = "A";
std::string robot_id = "A";

#define START std::string("START----")
#define STOP  std::string("STOP-----")
#define PING  std::string("PING-----")
#define ACK   std::string("ACK------")

#define ALL_SIGNAL(field)            std::string("a") + std::string(field) + std::string("X")
#define SIGNAL(field, robot_letter)  std::string("a") + std::string(field) +  std::string(robot_letter)

#define RECEIVE_TAG  std::string("ref:")
#define SEND_TAG     std::string("rf:")

#define START_SIGNAL_ALL(field)            (RECEIVE_TAG + ALL_SIGNAL(field)           + START)
#define START_SIGNAL(field, robot_letter)  (RECEIVE_TAG + SIGNAL(field, robot_letter) + START)

#define STOP_SIGNAL_ALL(field)             (RECEIVE_TAG + ALL_SIGNAL(field)           + STOP)
#define STOP_SIGNAL(field, robot_letter)   (RECEIVE_TAG + SIGNAL(field, robot_letter) + STOP)

#define PING_SIGNAL(field, robot_letter)   (RECEIVE_TAG + SIGNAL(field, robot_letter) + PING)

#define ACK_SIGNAL(field, robot_letter)    (SEND_TAG    + SIGNAL(field, robot_letter) + ACK)


void serial_init(int* serial){

  struct termios tty, tty_old;
  memset(&tty, 0, sizeof tty);

  /* Error Handling */
  if(tcgetattr(*serial, &tty) != 0)
    std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;

  /* Save old tty parameters */
  tty_old = tty;

  /* Set Baud Rate */
  cfsetospeed(&tty, (speed_t)B115200);
  cfsetispeed(&tty, (speed_t)B115200);

  /* Setting other Port Stuff */
  // Disable parity bit
  tty.c_cflag &= ~PARENB;

  // Default stop bits
  tty.c_cflag &= ~CSTOPB;

  // Serial byte is 8 bits
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  // No flow control
  tty.c_cflag &= ~CRTSCTS;

  // Enable Read and ignore control lines
  tty.c_cflag |= CREAD | CLOCAL;

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then apply attributes */
  tcflush(*serial, TCIFLUSH);

  if(tcsetattr(*serial, TCSANOW, &tty) != 0)
    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
}

std::string message_buffer;
int serial_port;

/**
 * If serial node gets a Command message
 */
void command_handler(const core::Command::ConstPtr& msg){
  std::string command = msg->command;

  // inlined write_command so no one would have to use self-restraint to not call it without publishing to the topic
  std::string write_string = command + "\n";
  ROS_DEBUG_STREAM(write_string);
  write(serial_port, write_string.c_str(), write_string.size());
  //std::cout << "sending: " << command << "\n";
  //ROS_INFO("wrote");
}

ros::Publisher command_topic_out;
void send_cmd(std::string cmd) {
  core::Command command;
  command.command = cmd;
  command_topic_out.publish(command);
}

void send_for_gs(const ros::TimerEvent&){
  send_cmd("gs");
}




// removes the message tags in the front and back
std::string remove_tags(std::string str) {
  size_t last = str.find('>');

  if (str[0] == '<' && last != std::string::npos ) {
    return str.substr(1, last-1);
  }
  return "";
}

std::string add_tags(std::string str){
  return std::string("<") + str + std::string(">");
}

// Mimicking pythons split function
std::vector<std::string> split(std::string str, std::string sep = " ") {
  std::vector<std::string> result;

  size_t start = 0;
  size_t end   = str.find(sep);
  while(end != std::string::npos) {
    result.push_back(str.substr(start, end-start));
    start = end + sep.length();
    end = str.find(sep, start);
  }
  result.push_back(str.substr(start, end));

  return result;
}


int main(int argc, char **argv){
  /* Serial code */
  std::cout << "Opening serial" << std::endl;



  // Open the serial port
  serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

  // Opening the serial port failed
  if(serial_port == -1){
    std::cout << strerror(errno) << std::endl;
    return -1;
  }

  std::cout << "Serial opened" << std::endl;

  /* Serial communcication */
  /* https://stackoverflow.com/a/18134892 */
  serial_init(&serial_port);
  

  /* ROS settings */
  ros::init(argc, argv, "serial");

  // Create an instance of ros NodeHandle
  ros::NodeHandle n;

  // Add the referee topic to topics pool
  ros::Publisher referee_topic_out = n.advertise<serial::Ref>("referee_signals", 128);
  ros::Publisher basket_out = n.advertise<std_msgs::String>("basket_type", 10);

  // Wheel speed topic
  ros::Publisher wheel_topic_out = n.advertise<serial::WheelSpeed>("wheelspeed", 128);

  // declared above
  command_topic_out = n.advertise<core::Command>("commands", 1000);

  // Subscribe to commands topic
  ros::Subscriber commands_topic_in = n.subscribe<core::Command>("commands", 1000, command_handler);

  // Timer for sending for gs
  ros::Timer timer = n.createTimer(ros::Duration(0.2), send_for_gs);
  
  ssize_t index = 0;
  char read_buf[512];

  // Set rate for updates
  ros::Rate r(80);

  // 12 sec



  while(ros::ok()){
    r.sleep();
    //send_cmd("gs");
    // what happens when we have something to read while at the same time, needing to write?
    // possible fix: write after reading.
    
    // There is something to write
    // if(command_in_buffer){
    //   write_cmd(command);
    //   command_in_buffer = false;
    // }

    // Waiting for Referee commands
    //else{

      // Attempt to read the serial port
      index = read(serial_port, &read_buf, 512);
      // When error occurred or nothing was read
      if(index <= 0 ){
        ros::spinOnce();
        //std::cout << "Serial read error" << "\n";
        continue;
      }
      // Successful read
      else {

        //ROS_INFO("read");
        message_buffer.append(read_buf, index);
        
        // need to read more. no complete message
        size_t message_end = message_buffer.find('\n');
        if (message_end == std::string::npos) {
          ros::spinOnce();
          continue;
        }

        // extract one message from the buffer
        std::string message_tagged = message_buffer.substr(0, message_end);
        message_buffer = message_buffer.substr(message_end+1);

        if (message_tagged == "") {
          ros::spinOnce();
          continue;
        }


        std::string message = remove_tags(message_tagged);
        if (message == "") {
          
        }    

        /* Check if received message was referee signal */
        if(message.find("ref") != std::string::npos) {
          // Message object to be sent to referee topic
          serial::Ref msg;

          // if ping: no ref
          // if all: no ack

          // Robot received start signal
          if(message == (START_SIGNAL(field_id, robot_id))) {
            send_cmd(ACK_SIGNAL(field_id, robot_id));
            msg.start = true;
            referee_topic_out.publish(msg);
          }
          // Robot received stop signal
          else if(message == (STOP_SIGNAL(field_id, robot_id))) {
            msg.start = false;
            send_cmd(ACK_SIGNAL(field_id, robot_id));
            referee_topic_out.publish(msg);
          }
          // Robot received ping signal, send ACK
          else if(message == (PING_SIGNAL(field_id, robot_id))) {
            send_cmd(ACK_SIGNAL(field_id, robot_id));
          }
          //
          else if(message == (STOP_SIGNAL_ALL(field_id))){
            msg.start = false;
            referee_topic_out.publish(msg);
          }
          //
          else if(message == (START_SIGNAL_ALL(field_id))){
            msg.start = true;
            referee_topic_out.publish(msg);
          }
        }
        
        // Robot received wheel rotation
        else if (message.find("gs") != std::string::npos) {
          serial::WheelSpeed speeds;
          const std::vector<std::string> split_message = split(message, ":");
          
          // split_message should be for example [gs, 5, 100, -20]
          speeds.wheel1 = stoi(split_message[1]);
          speeds.wheel2 = stoi(split_message[2]);
          speeds.wheel3 = stoi(split_message[3]);
          speeds.wheel4 = stoi(split_message[4]);
          wheel_topic_out.publish(speeds);
        } else if (message.find("stop") != std::string::npos) {
          // TODO
        } else if (message.find("remote") != std::string::npos) {
          // TODO
        } else if (message.find("robot_id") != std::string::npos) {
          robot_id = split(message, ":")[1];
        } else if (message.find("field_id") != std::string::npos) {
          field_id = split(message, ":")[1];
        } else if (message.find("basket") != std::string::npos) {
          std::string basket = split(message, ":")[1];
          //ROS_INFO("basket: %s", basket.c_str());
          std_msgs::String msg;
          msg.data = basket;
          basket_out.publish(msg);
        }

        // Received message is unknown
        else {
          ROS_INFO("RECEIVED: %s", message.c_str());
          continue;
        }

        // Publish the message to referee topic
      //}

      // Refresh loop variables
      index = 0;


    }

    // For callbacks
    ros::spinOnce();

  }

  return 0;
}
