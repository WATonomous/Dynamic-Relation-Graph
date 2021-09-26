#include "pp_env_model/behavioral/stop_sign_state.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

StopSignState::StopSignState(StopSign sign, StopSignStateAction action):
    AbstractState(action), _sign{sign}, _ego_state{EgoStateAction::WAITING_FOR_COMMANDS} {}

std::string StopSignState::serializeAction() const {
    switch(_action) {
        case StopSignStateAction::NON_BLOCKING:
            return "NON_BLOCKING";
        case StopSignStateAction::BLOCKING:
            return "BLOCKING";
        case StopSignStateAction::ACTIVE_BLOCKING:
            return "ACTIVE_BLOCKING";
        case StopSignStateAction::INACTIVE:
            return "INACTIVE";
        default:
            return "UNKNOWN, update StopSignState::serializeAction()";
    }
}

std::string StopSignState::serialize() const {
    return "";
}

StopSignState StopSignState::setSign(const StopSign& sign) {
    _sign = sign;
    return *this;
}

const StopSign& StopSignState::getSign() const {
    return _sign;
}

StopSignState StopSignState::setEgoState(const EgoState& ego_state) {
    _ego_state = ego_state;
    return *this;
}

const EgoState& StopSignState::getEgoState() const {
    return _ego_state;
} 

StopSignState StopSignState::setStopResult(const TrafficRegulationStopResult &result) {
    _stop_result = result;
    return *this;
}

const TrafficRegulationStopResult& StopSignState::getStopResult() const {
    return _stop_result;
}

StopSignState StopSignState::setActivatedTime(ros::Time time) {
    _activated_time = time;
    return *this;
}

ros::Time StopSignState::getActivatedTime() const {
    return _activated_time;
}

void StopSignState::updateStopResult() {
    if (_ego_state.getPathResult().hasPath() && _action != StopSignStateAction::NON_BLOCKING) {
        _stop_result.updateDistance(_ego_state.getPathResult().getPath().get().front());
    }
}