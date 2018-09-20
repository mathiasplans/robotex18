#include <ros/ros.h>
#include <serial/Ref.msg>
#include <core/Command.msg>

#include <sstream>
#include <string>

/**
 *
 */
#define START_SIGNAL  std::string("START")
#define STOP_SIGNAL   std::string("STOP")
#define PING_SIGNAL   std::string("PING")

#define ACK_SIGNAL    std::string("ACK")


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

  // Non-blocking read
  tty.c_cc[VMIN] = 1;

  // Read timeout (0.5s)
  tty.c_cc[VTIME] = 5;

  // Enable Read and ignore control lines
  tty.c_cflag |= CREAD | CLOCAL;

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then apply attributes */
  tcflush(*serial, TCIFLUSH );

  if(tcsetattr(*serial, TCSANOW, &tty) != 0)
    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
}

std::string command, referee;
bool command_in_buffer = false;

/**
 *
 */
void command_handler(const std_msgs::Command::ConstPtr& msg){
  command = msg->command;
  command_in_buffer = true;
}

int main(int argc, char **argv){
  /* Serial code */
  std::cout << "Opening serial" << std::endl;

  // Open the serial port
  serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

  // Opening the serial port failed
  if(serial_port == -1){
    std::cout << strerror(errno) << std::endl;
    return serial;
  }

  std::cout << "Serial opened" << std::endl;

  /* Serial communcication */
  /* https://stackoverflow.com/a/18134892 */
  serial_init(&serial_port);

  /* ROS settings */
  ros::init(argc, argv, "serial")

  // Create an instance of ros NodeHandle
  ros::NodeHandle n;

  // Add the referee topic to topics pool
  ros::Publisher referee_topic_out = n.advertise<std_msgs::Ref>("referee_signals", 1000);

  // Subscribe to commands topic
  ros::Subscriber commands_topic_in = n.subscribe("commands", 1000, command_handler);

  // Add Message publishing rate
  ros::Rate loop_rate(10);

  int16_t n = 0, spot = 0;
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
          n = read(serial_port, &buf, 1);
          referee.push_back(buf);
      }while(buf != '\r' && n > 0);

      // When error occurred
      if(n < 0)
        std::cout << "Error while reading from serial: " << strerror(errno) << std::endl;

      // Nothing was read
      else if(n == 0)
        continue;

      // Successful read
      else{
        // Message object, to be sent to referee topic
        std_msgs::Ref msg;

        /* Check if received message was referee signal */
        // Robot received start signal
        if(referee.compare(START_SIGNAL))
          msg.start = true;

        // Robot received stop signal
        else if(referee.compare(STOP_SIGNAL))
          msg.start = false;

        // Robot received ping signal, send ACK
        else if(referee.compare(PING_SIGNAL))
          write(serial_port, ACK_SIGNAL.c_str(), ACK_SIGNAL.size());

        // Received message was not a referee signal
        else
          std::cout << "Received serial message was not a referee signal:" << std::endl << "\t" << referee.c_str() << std::endl;
          continue;

        // Publish the message to referee topic
        std::cout << "Message from serial port: " << referee.c_str() << std::endl;
        referee_topic_out.publish(msg);
      }

      // Refresh loop variables
      n = spot = 0;

    }

    // For callbacks
    ros::spinOnce();

  }

  return 0;
}
