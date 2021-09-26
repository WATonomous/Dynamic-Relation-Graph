#include "pp_env_model/behavioral/pedestrian_trigger.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

/** 
 * Currently path planning is not given information about the crossing lanes in the world, 
 * so whenever a pedestrian comes close to our reference line, we stop for it, even if it is not 
 * near a crossing lane
*/
bool PedestrianInRoITrigger::test() {
    EgoState::PathResult pathResult = _state.getEgoState().getPathResult();

    if (!pathResult.hasPath()) {
        return false;
    }

    DynamicObjectStopResult stopResult = {pathResult.getFrenetReferenceLine().get(), _state.getEgoState().getPose(), _state.getPedestrianObj()};
    
    if (!stopResult.hasStop()) {
        return false;
    }

    stop_result = stopResult;

    return true;
}

bool PedestrianMovingTrigger::test() {
    return _state.getPedestrianObj().getVelocityMagnitude() >= Dynamic_Config.groups.behavioral_planning.OBJECT_MOVING_EPSILON;
}

bool PedestrianNotMovingTrigger::test() {
    return _state.getPedestrianObj().getVelocityMagnitude() < Dynamic_Config.groups.behavioral_planning.OBJECT_MOVING_EPSILON;
}

bool PedestrianNotInRoI::test() {
    return !_state.getStopResult().hasStop();
}

bool PedestrianStoppedTrigger::test() {
    ros::Duration pausedDuration = ros::Time::now() - _state.getPausedTime();
    ros::Duration pausedTime(Dynamic_Config.groups.behavioral_planning.OBJECT_STOPPED_TIMER);

    return pausedDuration >= pausedTime;
}
