#include "ros/ros.h"
#include "vision/Ball.h"
#include "vision/BasketRelative.h"
#include "serial/Ref.h"
#include "serial/WheelSpeed.h"

#include "core/Command.h"
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "debugcommands.hpp"
#include "wheelcontrol.hpp"
#include "statemachine.hpp"
#include "boost/bind.hpp"

#include "memory.hpp"
#include "speedcontrol.hpp"

// message includes
#include <localization/BasketAngle.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>

#include <poll.h>
#include <regex>

#define MAX_BALL_SHIFT 30
#define FORGET_TIME 4

/**
 *
 */
void ball_filter(float& ball_x, float& ball_y){
  static float last_ball_x = -1, last_ball_y = -1;
  static int error_count;

  std::cout << "ball_x: " << ball_x << " last_ball_x: " << last_ball_x << std::endl << "ball_y: " << ball_y << " last_ball_y: " << last_ball_y << std::endl;

  // Initialize the ball
  if(last_ball_x == -1 && ball_x != -1 && last_ball_y == -1 && ball_y != -1){
    last_ball_x = ball_x;
    last_ball_y = ball_y;
    return;
  }

  // If the ball position has chaned too much
  if(std::abs(ball_x - last_ball_x) > MAX_BALL_SHIFT * (error_count + 1) || std::abs(ball_y - last_ball_y) > MAX_BALL_SHIFT * (error_count + 1)){
    error_count += 1;
    ball_x = last_ball_x;
    ball_y = last_ball_y;

    if(error_count >= FORGET_TIME){
      error_count = 0;
      last_ball_x = last_ball_y = ball_x = ball_y = -1;
    }

    return;
  }

  // Reset the error counter
  else error_count = 0;

  // If error count is too high, dismiss the previous data and start again
  last_ball_x = ball_x;
  last_ball_y = ball_y;
}

/**
 * Handles the message from vision package
 */
void vision_callback_ball(const vision::Ball::ConstPtr& msg, StateMachine& sm){
  float ball_x = msg->ballX, ball_y = msg->ballY;

  //
  // ball_filter(ball_x, ball_y);

  //
  sm.update_ball_position(ball_x, ball_y, msg->width, msg->height);
}

void vision_callback_basket(const vision::Ball::ConstPtr& msg, StateMachine& sm){
  sm.update_basket_position(msg->ballX, msg->ballY, msg->width, msg->height);
}

void basket_depth_callback(const vision::BasketRelative::ConstPtr& msg, StateMachine& sm) {
  if(msg->depth > 10000){
    // sm.set_basket_dist(-1);
  }else{
    sm.set_basket_dist(msg->depth);
  }
}

/**
 * Handle the message from serial package
 */
void referee_handler(const serial::Ref::ConstPtr& msg, StateMachine& sm){
  if(msg->start)
    sm.start_machine();
  else
    sm.stop_machine();
}

void handle_basket(const std_msgs::String::ConstPtr& msg, StateMachine& sm) {
    if (msg->data == "pink") {
        sm.set_primary_basket(PINK);
    } else if (msg->data == "blue") {
        sm.set_primary_basket(BLUE);
    } else {
        ROS_ERROR("Tried to set basket other than (blue|pink)");
    }
}

static Memory mem = Memory();

/**
 *
 */
void localization_callback(const localization::BasketAngle::ConstPtr& msg, StateMachine& sm){
  if(sm.blue_is_primary()){
    sm.set_basket_angle(msg->blue, msg->pink);
  }else{
    sm.set_basket_angle(msg->pink, msg->blue);
  }
}

/**
 *
 */
void localization_position_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, StateMachine& sm){
  // Blue is x positive
  // Pink is x negative
  // Right from blue is y positive
  // Left from blue is y negative
  // Basket to center - 2.1m
  // Court length 4.6m
  // Court width 2m
  float basket_x, basket_y = 0;
  if(sm.blue_is_primary()){
    basket_x = 2.6f;
  }else{
    basket_x = -2.6f;
  }

  // float distance_from_basket = std::sqrt(std::pow(basket_x - msg.pose.pose.position.x, 2) + std::pow(basket_y - msg.pose.pose.position.y, 2));
}


void handle_odometry(const nav_msgs::Odometry::ConstPtr& msg, StateMachine& sm) {

  tf::Quaternion q_orig;
  quaternionMsgToTF(msg->pose.pose.orientation , q_orig);

  double roll, pitch, yaw; // roll and pitch should be 0 all the time
  tf::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

  sm.update_robot_position(
          msg->pose.pose.position.x,
          msg->pose.pose.position.y,
          yaw
          );
}

int main(int argc, char **argv){

  // Initialize ROS
  ros::init(argc, argv, "core");

  // Handle for the specific node
  ros::NodeHandle n;

  // Add the referee topic to topics pool
  ros::Publisher command_topic_out = n.advertise<core::Command>("commands", 50);

  // Create state machine instance
  StateMachine sm = StateMachine(command_topic_out, n, BLUE);

  // Subscribe to a message from vision
  ros::Subscriber image_processor = n.subscribe<vision::Ball>("ball", 50, boost::bind(vision_callback_ball, _1, boost::ref(sm)));

  ros::Subscriber basket_sub = n.subscribe<vision::Ball>("basket", 50, boost::bind(vision_callback_basket, _1, boost::ref(sm)));

  ros::Subscriber basket_depth = n.subscribe<vision::BasketRelative>("basketrelative", 50, boost::bind(basket_depth_callback, _1, boost::ref(sm)));

  // Subscribe to a message from serial
  ros::Subscriber referee_signal = n.subscribe<serial::Ref>("referee_signals", 50, boost::bind(referee_handler, _1, boost::ref(sm)));


  ros::Subscriber loc_info = n.subscribe<localization::BasketAngle>("relativeangle", 50, boost::bind(localization_callback, _1, boost::ref(sm)));
  ros::Subscriber loc_pos = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("aruco_pose", 50, boost::bind(localization_position_callback, _1, boost::ref(sm)));
  ros::Subscriber odom = n.subscribe<nav_msgs::Odometry>("odom", 50, boost::bind(handle_odometry, _1, boost::ref(sm)));
  ros::Subscriber basket_type = n.subscribe<std_msgs::String>("basket_type", 10, boost::bind(handle_basket, _1, boost::ref(sm)));
          std::cout << "Init finished" << std::endl;

  // // Get the 'ball' rolling. Get it? no
  // core::Bob command;
  // command.ball = s.searching_for_ball();
  // bob.publish(command);

  /* Polling for stdin */
  pollfd cinfd[1];
  cinfd[0].fd = fileno(stdin);
  cinfd[0].events = POLLIN;

  while(ros::ok()){
    handle_debug_command(sm, cinfd);

    // Run the State Machine once
    sm.state_machine();
    ros::spinOnce();
  }

  return 0;
}
