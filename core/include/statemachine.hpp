#include <cstdint>

/**
 * Enum of all the states in state machine
 */
typedef enum{
  IDLE,             ///< The robot is idle, it doesn't do anything but tries to start searching for ball
  SEARCH_BALL,      ///< The robot is searching for balls
  CENTER_ON_BALL,   ///< Once the robot has found a suitable ball, center on it so it is in the middle of the camera's frame
  MOVE_TO_BALL,     ///< Once the ball is in the middle of the frame (Doesn't have to be if aproaching from an angle), move up to it
  SEARCH_BASKET,    ///< If the ball is sufficently close, search for the basket while keeping the ball in front of the robot
  THROW,            ///< Once the ball and the baslet are in the middle of the camera's frame, throw the ball to the basket
  CORRECT_POSITION  ///< If the basket is in the middle of the frame and the ball isn't, correct the positon
}state_t;

/**
 * The class for the main State Machine of the robot
 */
class StateMachine{
private:
  /* Machine control */
  bool stop_signal;         ///< If set, the robot will be set to and can not exit the IDLE state
  bool reset_signal;        ///< If set, the robot will be set to the IDLE state

  /* Internal variables for calculatng the position of the robot, basket, or balls */
  float object_position_x;  ///< X coordinates of the object
  float object_position_y;  ///< Y coordinates of the object
  bool object_in_sight;     ///< True if any objects are in sight
  float object_degrees_x = 0;

  /* Serial Communication */
  int serial;               ///< Handle of the serial port
  int counter = 0;          ///<

  /* State variables */
  state_t state = IDLE;     ///< The internal state of the state machine. For details, refer to state_t

  bool searching_ball = true;

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
   * The State Machine logic. Calling this functon will tick the state machine. Put this into the infinite loop
   */
  void state_machine(void);

  /**
   * Get the state of the State Machine
   */
  state_t get_state();

  /**
   * Initialize the State Machine
   */
  int init();

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
   * Update the position of the sought out object
   */
  void update_ball_position(
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

  /**
   * Returns true if searching for a ball, false if searching for a basket.
   */
  bool searching_for_ball();

};
