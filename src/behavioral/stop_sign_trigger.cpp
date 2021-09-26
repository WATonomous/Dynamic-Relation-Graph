#include "pp_env_model/behavioral/stop_sign_trigger.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

bool StopSignPresentOnRouteTrigger::test() {

    EgoState::PathResult path_result = _state.getEgoState().getPathResult();

    if (!path_result.hasPath()) {
        return false;
    }

    TrafficRegulationStopResult tryStop = {path_result.getPath().get().front()};

    if (!tryStop.hasStop()) {
        return false;
    }

    stop_result = tryStop;

    return true;
}

bool StopSignActivatedTrigger::test() {
    if (_state.getEgoState().getPose().distanceTo(_state.getStopResult().getStopPoint()) > Dynamic_Config.groups.behavioral_planning.STOP_SIGN_DISTANCE_EPSILON
        || _state.getEgoState().getPose().vel > 0.2) {
        return false;
    }

    return true;
}

bool StopSignDeactivatedTrigger::test() {
    if (_state.getEgoState().getPose().distanceTo(_state.getStopResult().getStopPoint()) >= Dynamic_Config.groups.behavioral_planning.STOP_SIGN_DISTANCE_EPSILON
        || _state.getEgoState().getPose().vel > 0.2) {
        return true;
    }

    return false;
}

bool StopSignExpiredTrigger::test() {
    ros::Duration activeDuration = ros::Time::now() - _state.getActivatedTime();
    ros::Duration stopTime(Dynamic_Config.groups.behavioral_planning.STOP_SIGN_TIME);

    return activeDuration >= stopTime;
}