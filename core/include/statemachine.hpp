#include <cstdint>
#include <ros/ros.h>

#include <string>

#include "lookup_table.hpp"

/**
 * Enum of all the states in state machine
 */
typedef enum{
  IDLE,              ///< The robot is idle, it doesn't do anything but tries to start searching for ball
  SEARCH_BALL,       ///< The robot is searching for balls
  CENTER_ON_BALL,    ///< Once the robot has found a suitable ball, center on it so it is in the middle of the camera's frame
  MOVE_TO_BALL,      ///< Once the ball is in the middle of the frame (Doesn't have to be if aproaching from an angle), move up to it
  SEARCH_BASKET,     ///< If the ball is sufficently close, search for the basket while keeping the ball in front of the robot
  THROW,             ///< Once the ball and the baslet are in the middle of the camera's frame, throw the ball to the basket
  CORRECT_POSITION,  ///< If the basket is in the middle of the frame and the ball isn't, correct the positon
  NUMBER_OF_STATES
}state_t;

/**
 * Sub-states for states in state_t
 */
typedef enum{
  BASKET_ORBIT_BALL,     ///< The robot orbits the ball until the basket is in sight
  BASKET_CENTER_BASKET,  ///< The robot adjusts it's position til the basket is in the middle of a frame
  BASKET_ORBIT_BASKET,   ///< The robot orbits the basket until the ball is in the middle of the frame
  THROW_AIM,             ///< Robot aims the thrower and get's the best throwing power
  THROW_GOAL,            ///< Robot moves forward and throws the ball
  THROW_GOAL_NO_BALL,    ///< Robot throws the ball even if ball is not in sight. Usually,
                         ///< at the end of the throw, the ball is not in sight,
                         ///< even if it's not throwh yet. To avoid not throwing the ball,
                         ///< the robot has to continue throwing even if ball is not in sight.
                         ///< After a brief time, the robot will exit this state
                         ///< and return to searching the ball.
  THROW_DEAIM            ///< Should the robot be finished with throwing,
                         ///< it returns the thrower to it's default state
}substate_t;

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
  float object_position_x;  ///< X coordinates of the object
  float object_position_y;  ///< Y coordinates of the object
  float basket_position_x;  ///< X coordinates of the basket
  float basket_position_y;  ///< Y coordinates of the basket
  bool object_in_sight = false;     ///< True if any objects are in sight
  bool basket_in_sight;
  bool basket_found = false;
  bool ball_in_sight = false;

  /* Aiming variables */
  uint16_t aimer_position;  ///< The position of the aimer, determines the arc of the throw
  uint16_t thrower_power;   ///< How strongly does the motor on the thrower work. Ranges from 1001 to 2000

  /* Serial Communication */
  int serial;                  ///< Handle of the serial port

  /* State variables */
  state_t state = IDLE;        ///< The internal state of the state machine. For details, refer to state_t
  substate_t substate[NUMBER_OF_STATES];

  /* ROS variables */
  ros::Publisher publisher;    ///< Publisher object for serial node
  ros::Rate command_delay;     ///< Delay between sending the commands

  /* Misc variables */
  bool searching_ball = true;  ///< True if robot requires information about ball position

  /* Communication functions */
  /**
   * Function for communicating with serial node.
   */
  void serial_write(
    std::string  ///< [in] String to be sent over serial
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
   * Constructor with publisher object, use this if you want the object to communicate with serial port
   */
  StateMachine(ros::Publisher&);

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
    state_t superstate  ///< [in] Superstate whom substate is called upon
  );

  /**
   * Change the state of the State Machine
   */
  void set_state(
    state_t superstate  ///< [in] State will be set to this
  );

  /**
   * Change the sub-state of a state of the State Machine
   */
  void set_substate(
    state_t superstate,      ///< [in] Superstate whom substate will be set
    substate_t new_substate  ///< [in] Sub-state will be set to this
  );

  /**
   * Resets all substates to default
   */
  void reset_substates();

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
    throw_parameters_t& throw_parameters  ///< Struct which contains the aimer arc and thrower power (.aim and .thrower)
  );

  /**
   * Update the position of the sought out object
   */
  void update_ball_position(
    int16_t x,       ///< [in] X coordinates of the object
    int16_t y,       ///< [in] Y coordinates of the object
    uint16_t width,  ///< [in] Width of the camera's frame
    uint16_t height  ///< [in] Height of the camera's frame
  );

  void update_basket_position(
    int16_t x,       ///< [in] X coordinates of the object
    int16_t y,       ///< [in] Y coordinates of the object
    uint16_t width,  ///< [in] Width of the camera's frame
    uint16_t height  ///< [in] Height of the camera's frame
  );

  /**
   * Lets the State Machine know if any objects are in sight (Balls, baskets, etc.)
   */
  void set_object_in_sight(
    bool in_sight  ///< [in] True if something is in sight
  );

  void set_basket_in_sight(
    bool in_sight  ///< [in] True if something is in sight
  );

  /**
   * Returns true if searching for a ball, false if searching for a basket.
   */
  bool searching_for_ball();

  /**
   * Setter for stop signal.
   * If stop is set to true, the machine is on hold.
   * If stop is set to false, the machine behaves regularly
   */
  void set_stop_signal(bool);

};
