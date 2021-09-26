#include "pp_env_model/behavioral/global_command_state.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

GlobalCommandState::GlobalCommandState(GlobalCommandStateAction action):
    AbstractState(action), _ego_state{EgoStateAction::WAITING_FOR_COMMANDS} {}

std::string GlobalCommandState::serializeAction() const {
    switch(_action) {
        case GlobalCommandStateAction::TURNING_RIGHT:
            return "TURNING_RIGHT";
        case GlobalCommandStateAction::TURNING_LEFT:
            return "TURNING_LEFT";
        case GlobalCommandStateAction::HEADING_STRAIGHT:
            return "HEADING_STRAIGHT";
        case GlobalCommandStateAction::STOPPING:
            return "STOPPING";
        case GlobalCommandStateAction::WAITING_FOR_GLOBAL_COMMAND:
            return "WAITING_FOR_GLOBAL_COMMAND";
        case GlobalCommandStateAction::DONE:
            return "DONE";
        default:
            return "UNKNOWN, update GlobalCommandState::serializeAction()";
    }
}

std::string GlobalCommandState::serialize() const {
    return "";
}

GlobalCommandState GlobalCommandState::setGlobalCommandQueue(const path_planning::GlobalCommandQueue &cmds) {
    _global_cmd_queue = cmds;
    return *this;
}

path_planning::GlobalCommandQueue GlobalCommandState::getGlobalCommandQueue() const {
    return _global_cmd_queue;
}

GlobalCommandState GlobalCommandState::setEgoState(const EgoState &ego_state){
    _ego_state = ego_state; 
    return *this; 
}

EgoState GlobalCommandState::getEgoState() const {
    return _ego_state; 
}

