#include "pp_env_model/behavioral/global_command_trigger.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

bool TriggerTurnRight::test() {
    if(_state.getGlobalCommandQueue().empty()) return false;

    return _state.getGlobalCommandQueue().front().cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::RIGHT_TURN;
}

bool TriggerTurnLeft::test() {
    if(_state.getGlobalCommandQueue().empty()) return false;

    return _state.getGlobalCommandQueue().front().cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::LEFT_TURN;
}

bool TriggerStop::test() {
    if(_state.getGlobalCommandQueue().empty()) return false;

    return _state.getGlobalCommandQueue().front().cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::STOP;
}

bool TriggerHeadStraight::test() {
    if(_state.getGlobalCommandQueue().empty()) return false;

    return _state.getGlobalCommandQueue().front().cmd == path_planning::GlobalCommand::INSTRUCTION_TYPE::STRAIGHT;
}

bool TriggerGlobalCommandComplete::test() {
    EgoState ego_state = _state.getEgoState();
    auto pathResult = ego_state.getPathResult();
    GlobalCommandQueue q = _state.getGlobalCommandQueue();
    if (!pathResult.hasPath() || q.empty()) {
        return false;
    }
    int currLaneletID = pathResult.getPath().get()[0].id;
    GlobalCommand nextCmd = q.front();
    if (nextCmd.nearIntersection(ego_state.getPose(), Dynamic_Config.safe_intersection_distance)) {
        return false;
    }

    return nextCmd.isComplete(ego_state.getPose(), currLaneletID);
}

bool TriggerNoGlobalCommand::test() {
    return _state.getGlobalCommandQueue().empty();   
}

bool TriggerHasGlobalCommand::test() {
    return !_state.getGlobalCommandQueue().empty(); 
}