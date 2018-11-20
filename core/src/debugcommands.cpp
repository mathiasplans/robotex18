#include "debugcommands.hpp"
#include <string>
#include <regex>
#include <vector>

// Mimicking pythons split function
std::vector<std::string> split(std::string str, std::string sep = " ") {
  std::vector<std::string> result;

  size_t start = 0;
  size_t end   = str.find(sep);
  while(end != std::string::npos) {
    result.push_back(str.substr(start, end-start));
    start = end + sep.length();
    end = str.find(sep, start);
  }
  result.push_back(str.substr(start, end));
  return result;
}

/**
 *
 */
void handle_debug_command(StateMachine& sm, pollfd* cinfd){
  static std::string input_command;

  // Return if nothing is on the command line
  if(!poll(cinfd, 1, 1)) return;

  // Handle the input commands
  if(std::getline(std::cin, input_command)){
    if(input_command == std::string("start")){
      sm.start_machine();
      std::cout << "The robot has been started" << std::endl;
    }
    else if(input_command == std::string("stop")){
      sm.stop_machine();
      std::cout << "The robot has been  stopped" << std::endl;
    }
    else if(input_command == std::string("reset")){
      sm.reset_machine();
      std::cout << "The robot has been reset" << std::endl;
    }
    else if(input_command == std::string("pause")){
      sm.pause_machine();
      std::cout << "The robot has been paused" << std::endl;
    }
    // Currently doesn't work
    else if(std::regex_search(input_command, std::regex("set state [A-Z_]+"))){
      std::string state_string = split(input_command)[2];

      std::cout << "Switching to state: " << state_string << std::endl;
      if(state_string == "IDLE") sm.set_state(IDLE);
      else if(state_string == "SEARCH_BALL") sm.set_state(SEARCH_BALL);
      else if(state_string == "MOVE_TO_BALL") sm.set_state(MOVE_TO_BALL);
      else if(state_string == "SEARCH_BASKET") sm.set_state(SEARCH_BASKET);
      else if(state_string == "THROW") sm.set_state(THROW);
      else std::cout << "Entered state is invalid" << std::endl;
    }
    // Currently doesn't work
    else if(std::regex_search(input_command, std::regex("set substate [A-Z_]+"))){
      std::string substate_string = split(input_command)[2];

      std::cout << "Swithcing to substate: " << substate_string << std::endl;
      if(substate_string == "THROW_AIM") sm.set_substate(THROW, THROW_AIM);
      else if(substate_string == "THROW_GOAL") sm.set_substate(THROW, THROW_GOAL);
      else if(substate_string == "THROW_GOAL_NO_BALL") sm.set_substate(THROW, THROW_GOAL_NO_BALL);
      else std::cout << "Entered substate is invalid" << std::endl;
    }
    else if(input_command == "reset substates"){
      sm.reset_substates();
      std::cout << "The substates have been set to their default value" << std::endl;
    }
    else if(input_command == "get state") std::cout << "The State Machine is in " << std::to_string(sm.get_state()) << " state" << std::endl;
    else if(std::regex_search(input_command, std::regex("set thrower \\d+"))){
      std::string thrower_string = split(input_command)[2];
      sm.set_throw_power(std::stoi(thrower_string));
      std::cout << "Set the thrower to: " << thrower_string << std::endl;
    }
    else if(std::regex_search(input_command, std::regex("set aimer \\d+"))){
      std::string aimer_string = split(input_command)[2];
      sm.set_aimer_position(std::stoi(aimer_string));
      std::cout << "Set the aimer to: " << aimer_string << std::endl;
    }
    else std::cout << "Entered command is invalid" << std::endl;

    // Clear the string for the new commands to be read
    input_command.clear();
  }
}
