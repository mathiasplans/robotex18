#include <poll.h>
#include <statemachine.hpp>

/**
 * Handles commands from commandline
 */
void handle_debug_command(
  StateMachine& sm,  ///< Reference to a state machine object, throuh which the commands are executed
  pollfd* cinfd      ///< std::cin file descriptor, will be polled and read
);
