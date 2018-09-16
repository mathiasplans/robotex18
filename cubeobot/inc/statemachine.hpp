#include <thread>

/**
 *
 */
typedef enum{
  IDLE,             ///<
  SEARCH_BALL,      ///<
  SEARCH_BASKET,    ///<
  MOVE_TO_BALL,     ///<
  THROW             ///< NOTE: MOVE_TO_THROW and THROW should perhaps merge?
}state_t;

class StateMachine{
private:
  /* The main thread */
  // Thread Handle
  std::thread state_thread;
  // Thread Task
  void state_machine(void);

  // Machine control
  bool stop_signal;

  /*  */
  bool search_for_ball();
  bool center_on_ball();
  bool throw_the_ball();
  bool goto_ball();
  bool search_for_basket();

  /*  */
  float object_position;
  bool object_in_sight;

  /* Serial Communication */
  int serial;

public:

  StateMachine();
  state_t get_state();

};
