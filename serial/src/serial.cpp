#include <ros/ros.h>
#include <serial/Ref.h>
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
#define DB_FIELD     A
#define DB_ROBOT_NO  A

#define START std::string("START---->")
#define STOP  std::string("STOP----->")
#define PING  std::string("PING----->")
#define ACK   std::string("ACK------>")

#define ALL_SIGNAL(field)            std::string("<ref:a") + std::string(field) + std::string("X")
#define SIGNAL(field, robot_letter)  std::string("<ref:a") + std::string(field) +  std::string(robot_letter)


#define START_SIGNAL_ALL(field)            ALL_SIGNAL(field)           + START
#define START_SIGNAL(field, robot_letter)  SIGNAL(field, robot_letter) + START

#define STOP_SIGNAL_ALL(field)            ALL_SIGNAL(field)           + STOP
#define STOP_SIGNAL(field, robot_letter)  SIGNAL(field, robot_letter) + STOP

#define PING_SIGNAL(field, robot_letter)  SIGNAL(field, robot_letter) + PING

#define ACK_SIGNAL(field, robot_letter)    SIGNAL(field, robot_letter) + ACK


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

std::string command, referee;
bool command_in_buffer = false;

/**
 * If serial node gets a Command message
 */
void command_handler(const core::Command::ConstPtr& msg){
  command = msg->command;
  command_in_buffer = true;
}

int main(int argc, char **argv){
  /* Serial code */
  std::cout << "Opening serial" << std::endl;

  // Open the serial port
  int serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);

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
  ros::Publisher referee_topic_out = n.advertise<serial::Ref>("referee_signals", 1000);

  // Subscribe to commands topic
  ros::Subscriber commands_topic_in = n.subscribe<core::Command>("commands", 1000, command_handler);

  int16_t index = 0, spot = 0;
  char buf = '\0';

  while(ros::ok()){
    // There is something to write
    if(command_in_buffer){
      write(serial_port, command.c_str(), command.size());
      command_in_buffer = false;
    }

    // Waiting for Referee commands
    else{
      // Clear the string before each event
      referee.clear();

      // Attempt to read the serial port
      do{
          index = read(serial_port, &buf, 1);
          referee.push_back(buf);
      }while(buf != '\r' && index > 0);

      // When error occurred
      if(index < 0){
        ros::spinOnce();
        continue;
      }

      // Nothing was read
      else if(index == 0){
        ros::spinOnce();
        continue;
      }
      // Successful read
      else{
        // Message object to be sent to referee topic
        serial::Ref msg;

        /* Check if received message was referee signal */
        // Robot received start signal
        if(referee.compare(START_SIGNAL(DB_FIELD, DB_ROBOT_NO)))
          msg.start = true;

        // Robot received stop signal
        else if(referee.compare(STOP_SIGNAL(DB_FIELD, DB_ROBOT_NO)))
          msg.start = false;

        // Robot received ping signal, send ACK
        else if(referee.compare(PING_SIGNAL(DB_FIELD, DB_ROBOT_NO)))
          write(serial_port, ACK_SIGNAL(DB_FIELD, DB_ROBOT_NO).c_str(), ACK_SIGNAL(DB_FIELD, DB_ROBOT_NO).size());

        // Received message was not a referee signal
        else
          std::cout << "Received serial message was not a referee signal:" << std::endl << "\t" << referee.c_str() << std::endl;
          continue;

        // Publish the message to referee topic
        std::cout << "Message from serial port: " << referee.c_str() << std::endl;
        referee_topic_out.publish(msg);
      }

      // Refresh loop variables
      index = spot = 0;

    }

    // For callbacks
    ros::spinOnce();

  }

  return 0;
}
