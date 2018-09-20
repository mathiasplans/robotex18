#include <cstdint>
#include <ros/ros.h>

/**
 *
 */
typedef enum{
  IDLE,             ///<
  SEARCH_BALL,      ///<
  SEARCH_BASKET,    ///<
  MOVE_TO_BALL,     ///<
  THROW             ///<
}state_t;

class StateMachine{
private:
  /* The main thread */

  // Machine control
  bool stop_signal;

  /* Navigation functions */
  bool search_for_ball();
  bool center_on_ball();
  bool throw_the_ball();
  bool goto_ball();
  bool search_for_basket();

  /*  */
  float object_position;
  bool object_in_sight;

  state_t state = IDLE;

  ros::Publish publisher;
  void serial_write();

public:
  StateMachine(ros::Publish);

  int init();

  void state_machine(void);

  void update_ball_position(int32_t, int32_t);
  void set_object_in_sight(bool);

  void set_stop_signal(bool);

};
