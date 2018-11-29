#pragma once
#include <cstdint>
#include <ros/ros.h>

#include <string>

#include "defines.hpp"
#include "lookup_table.hpp"

/**
 * Enum of all the states in state machine
 */
typedef enum{
  IDLE,              ///< The robot is idle, it doesn't do anything but tries to start searching for ball
  SEARCH_BALL,       ///< The robot is searching for balls
  REPOSITION,        ///< If there aren't any balls in sight, reposition to another location for better coverage
  MOVE_TO_BALL,      ///< Once the ball is in the middle of the frame (Doesn't have to be if aproaching from an angle), move up to it
  SEARCH_BASKET,     ///< If the ball is sufficently close, search for the basket while keeping the ball in front of the robot
  THROW,             ///< Once the ball and the baslet are in the middle of the camera's frame, throw the ball to the basket
  NUMBER_OF_STATES,
  TEST
}state_t;

static std::array<std::string, 8> state_names = {
        "idle",
        "search_ball",
        "reposition",
        "move_to_ball",
        "search_basket",
        "throw",
        "number_of_states",
        "test"
};

/**
 * Sub-states for states in state_t
 */
typedef enum{
  THROW_AIM,             ///< Robot aims the thrower and get's the best throwing power
  THROW_GOAL,            ///< Robot moves forward and throws the ball
  THROW_GOAL_NO_BALL     ///< Robot throws the ball even if ball is not in sight. Usually,
                         ///< at the end of the throw, the ball is not in sight,
                         ///< even if it's not throwh yet. To avoid not throwing the ball,
                         ///< the robot has to continue throwing even if ball is not in sight.
                         ///< After a brief time, the robot will exit this state
                         ///< and return to searching the ball.
}substate_t;

/**
 *
 */
typedef enum{
  BLUE,  ///<
  PINK   ///<
}basket_t;

/**
 * The class for the main State Machine of the robot
 */
class StateMachine{
private:
  /* Machine control */
  bool stop_signal = true;    ///< If set, the robot will be set to and can not exit the IDLE state
  bool reset_signal = false;  ///< If set, the robot will be set to the IDLE state
  bool pause_signal = false;  ///< If set, the robot will be paused, state and substate won't be affected

  /* Internal variables for calculatng the position of the robot, basket, or balls */
  float ball_position_x;  ///< X coordinates of the ball
  float ball_position_y;  ///< Y coordinates of the ball
  float basket_position_x;  ///< X coordinates of the basket
  float basket_position_y;  ///< Y coordinates of the basket
  float basket_angle_primary;  ///< Relative angle to the blue basket, a.k.a angle between robot's peripheral view and the basket
  float basket_angle_secondary;  ///< Relative angle to the pink basket, a.k.a angle between robot's peripheral view and the basket
  basket_t primary_basket;  ///< The type of target basket. Can be BLUE or PINK

  /* Variables which determine the decisions of the robot */
  bool throw_completed = false;  ///< True if a throw was a success

  /* Aiming variables */
  int basket_dist = -1;

  /* Serial Communication */
  int serial;  ///< Handle of the serial port

  /* State variables */
  state_t state = IDLE;                   ///< The internal state of the state machine. For details, refer to state_t
  substate_t substate[NUMBER_OF_STATES];  ///< An array of substates. The superstates are the indices of this array.

  /* ROS variables */
  ros::Publisher publisher;    ///< Publisher object for serial node
  ros::Rate command_delay;     ///< Delay between sending the commands
  ros::NodeHandle& ros_node;   ///< Reference to ROS Node Handle object
  ros::Timer throwing_timer;   ///< Timer for throwing with no ball in frame

  /* Internal state functions */
  /**
   * This will notify the machine that it's OK to start searching
   * for a ball again and that the preceding throw was completed
   *
   * Sets the throw_completed to true.
   *
   * It is vital that after reading the throw_completed, it should be reset back to false
   */
  void complete_throw(const ros::TimerEvent&);



  /* Lookup table functions */
  /**
   * Looks up the thrower configuration according to the distace from lookup table
   */
  throw_info_t look_up(
    int distance  ///< Distance between a ball and a basket
  );

  /* Control of the movement and actions */
  /**
   * Give a command to main board to move the robot to search for a ball.
   * Returns true when the task is complete
   */
  bool search_for_ball();

  /**
   * Give a command to main board to move the robot to center on the ball
   * Returns true when the task is complete
   */
  bool center_on_ball();

  /**
   * Give a command to main board to throw the ball
   * Returns true when the task is complete
   */
  bool throw_the_ball();

  /**
   * Give a command to main board to move up to the ball
   * Returns true when the task is complete
   */
  bool goto_ball();

  /**
   * Give a command to main board to search for a basket
   * Returns true when the task is complete
   */
  bool search_for_basket();

public:
  /**
   * Main constructor of the State Machine class
   */
  StateMachine();

  /**
   * Constructor with publisher object, use this if you want
   * the object to communicate with serial port or be functional at all
   */
  StateMachine(ros::Publisher&, ros::NodeHandle&, basket_t basket_type);

  /**
   * The State Machine logic. Calling this functon will tick the state machine. Put this into the infinite loop
   */
  void state_machine(void);

  /**
   * Get the state of the State Machine
   */
  state_t get_state();

  /**
   * Get the sub-state of the state of the State Machine
   */
  substate_t get_substate(
    state_t superstate  ///< Superstate whom substate is called upon
  );

    /* Communication functions */
    /**
     * Function for communicating with serial node.
     */
    void serial_write(
            std::string  ///< String to be sent over serial
    );

  /**
   * Change the state of the State Machine
   */
  void set_state(
    state_t superstate  ///< State will be set to this
  );

  /**
   * Change the sub-state of a state of the State Machine
   */
  void set_substate(
    state_t superstate,      ///< Superstate whom substate will be set
    substate_t new_substate  ///< Sub-state will be set to this
  );

  /**
   * Resets all substates to default
   */
  void reset_substates();

  /**
   * Resets the minor state holders (throw_completed and the like)
   */
  void reset_internal_variables();

  /**
   * Reset the State Machine (Restarts from IDLE state)
   */
  void reset_machine();

  /**
   * Stop the machine (Holds in IDLE state)
   */
  void stop_machine();

  /**
   * Starts the machine (Can proceed from IDLE state)
   */
  void start_machine();

  /**
   * Pauses the operations of the State Machine
   * When start is called, the robot continues where
   * it left off when paused
   */
  void pause_machine();

  /**
   * Set the thrower power
   */
  void set_throw_power(
    uint16_t throw_pwr  ///< Power of the thrower, ranges from 1001 to 2000
  );

  /**
   * Set the aimer position
   */
  void set_aimer_position(
    uint16_t aimer_pos  ///< Position of the aimer, ranges from 1000 to 1800
  );

  /**
   * Set the variables before the throwing commences
   */
  void configure_thrower(
    const throw_info_t& throw_parameters  ///< Struct which contains the aimer arc and thrower power
  );

  /**
   * Resets the aimer and thrower to default value
   */
  void deaim();

  /**
   * Update the position of the sought out object
   */
  void update_ball_position(
    int x,       ///< X coordinates of the object
    int y,       ///< Y coordinates of the object
    int width,  ///< Width of the camera's frame
    int height  ///< Height of the camera's frame
  );

  void update_basket_position(
    int x,       ///< X coordinates of the object
    int y,       ///< Y coordinates of the object
    int width,  ///< Width of the camera's frame
    int height  ///< Height of the camera's frame
  );

  void set_basket_dist(
    int dist
  );

  /**
   * Setter for stop signal.
   * If stop is set to true, the machine is on hold.
   * If stop is set to false, the machine behaves regularly
   */
  void set_stop_signal(bool);

  /**
   *
   */
  void set_basket_angle(
      float blue_angle,  ///<
      float pink_angle   ///<
  );

  /**
   *
   */
  bool blue_is_primary();

  /**
   *
   */
  bool pink_is_primary();
};
