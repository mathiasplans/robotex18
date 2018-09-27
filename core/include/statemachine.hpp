#include <cstdint>

/**
 *
 */
typedef enum{
  IDLE,             ///<
  SEARCH_BALL,      ///<
  CENTER_ON_BALL,   ///<
  SEARCH_BASKET,    ///<
  MOVE_TO_BALL,     ///<
  THROW,            ///<
  CORRECT_POSITION  ///<
}state_t;

class StateMachine{
private:
  /* The main thread */

  // Machine control
  bool stop_signal;
  bool reset_signal;

  /*  */

  bool search_for_ball();
  bool center_on_ball();
  bool throw_the_ball();
  bool goto_ball();
  bool search_for_basket();

  /*  */
  float object_position_x;
  float object_position_y;
  bool object_in_sight;

  /* Serial Communication */
  int serial;
  int counter = 0;

  state_t state = IDLE;

public:
  void state_machine(void);
  StateMachine();
  state_t get_state();
  int init();

  void reset_machine();
  void stop_machine();
  void start_machine();

  void update_ball_position(int16_t, int16_t, uint16_t, uint16_t);
  void set_object_in_sight(bool);

};
