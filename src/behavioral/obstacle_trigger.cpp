#include "pp_env_model/behavioral/obstacle_trigger.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

bool ObstacleInRoITrigger::test() {
    EgoState::PathResult pathResult = _state.getEgoState().getPathResult();

    if (!pathResult.hasPath()) {
        return false;
    }

    DynamicObjectStopResult stopResult = {pathResult.getFrenetReferenceLine().get(), _state.getEgoState().getPose(), _state.getObstacle()};
    
    if (!stopResult.hasStop()) {
        return false;
    }

    stop_result = stopResult;

    return true;
}

bool ObstacleMovingTrigger::test() {
    return _state.getObstacle().getVelocityMagnitude() >= Dynamic_Config.groups.behavioral_planning.OBJECT_MOVING_EPSILON;
}

bool ObstacleStoppedTrigger::test() {
    return _state.getObstacle().getVelocityMagnitude() < Dynamic_Config.groups.behavioral_planning.OBJECT_MOVING_EPSILON;
}

bool ObstacleNotInRoI::test() {
    return !_state.getStopResult().hasStop();
}