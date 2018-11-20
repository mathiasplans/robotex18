#include <ros/ros.h>
#include <serial/Ref.h>
#include <serial/WheelSpeed.h>
#include <core/Command.h>

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
#define DB_FIELD     "A"
#define DB_ROBOT_NO  "A"

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

std::string command, message_buffer;
bool command_in_buffer = false;

/**
 * If serial node gets a Command message
 */
void command_handler(const core::Command::ConstPtr& msg){
  command = msg->command;
  command_in_buffer = true;
  //std::cout << "sending: " << command << "\n";
}

int serial_port;

void write_cmd(std::string cmd) {
  std::string write_string = cmd + "\r\n";
  write(serial_port, write_string.c_str(), write_string.size());
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
  serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);

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

  // Wheel speed topic
  ros::Publisher wheel_topic_out = n.advertise<serial::WheelSpeed>("wheelspeed", 128);

  // Subscribe to commands topic
  ros::Subscriber commands_topic_in = n.subscribe<core::Command>("commands", 1000, command_handler);

  ssize_t index = 0;
  char read_buf[32];

  // Set rate for updates
  ros::Rate r(100);

  while(ros::ok()){
    // what happens when we have something to read while at the same time, needing to write?
    // possible fix: write after reading.
    
    // There is something to write
    if(command_in_buffer){
      write_cmd(command);
      command_in_buffer = false;
    }

    // Waiting for Referee commands
    else{

      // Attempt to read the serial port
      index = read(serial_port, &read_buf, 32);

      // When error occurred or nothing was read
      if(index <= 0 ){
        ros::spinOnce();
        //std::cout << "Serial read error" << "\n";
        continue;
      }
      // Successful read
      else {
        
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

          // Robot received start signal
          if(message == (START_SIGNAL(DB_FIELD, DB_ROBOT_NO))) {
            msg.start = true;
            write_cmd(ACK_SIGNAL(DB_FIELD, DB_ROBOT_NO));
            referee_topic_out.publish(msg);
          }
          // Robot received stop signal
          else if(message == (STOP_SIGNAL(DB_FIELD, DB_ROBOT_NO))) {
            msg.start = false;
            write_cmd(ACK_SIGNAL(DB_FIELD, DB_ROBOT_NO));
            referee_topic_out.publish(msg);
          }
          // Robot received ping signal, send ACK
          else if(message == (PING_SIGNAL(DB_FIELD, DB_ROBOT_NO))) {
            write_cmd(ACK_SIGNAL(DB_FIELD, DB_ROBOT_NO));
          }
          //
          else if(message == (STOP_SIGNAL_ALL(DB_FIELD))){
            msg.start = false;
            referee_topic_out.publish(msg);
          }
          //
          else if(message == (START_SIGNAL_ALL(DB_FIELD))){
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
        }

        // Received message is unknown
        else {
          std::cout << "Received serial message was not a known command :" << std::endl << "\t" << message << std::endl;
          continue;
        }

        // Publish the message to referee topic
      }

      // Refresh loop variables
      index = 0;

    }

    // For callbacks
    ros::spinOnce();
  }

  return 0;
}
