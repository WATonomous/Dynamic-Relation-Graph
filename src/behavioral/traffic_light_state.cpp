#include "pp_env_model/behavioral/traffic_light_state.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

TrafficLightState::TrafficLightState(path_planning::TrafficLight light, TrafficLightStateAction action):
    AbstractState(action), _light{light}, _ego_state{EgoStateAction::WAITING_FOR_COMMANDS} {}

std::string TrafficLightState::serializeAction() const {
    switch(_action) {
        case TrafficLightStateAction::NON_BLOCKING:
            return "NON_BLOCKING";
        case TrafficLightStateAction::BLOCKING:
            return "BLOCKING";
        default:
            return "UNKNOWN, update TrafficLightState::serializeAction()";
    }
}

std::string TrafficLightState::serialize() const {
    return "";
}

TrafficLightState TrafficLightState::setLightStatus(path_planning::TrafficLight light) {
    _light = light;
    return *this;
}

path_planning::TrafficLight TrafficLightState::getLightStatus() const {
    return _light;
}

TrafficLightState TrafficLightState::setGlobalCommandQueue(path_planning::GlobalCommandQueue cmds) {
    _globalCommandQueue = cmds;
    return *this;
}

path_planning::GlobalCommandQueue TrafficLightState::getGlobalCommandQueue() const {
    return _globalCommandQueue;
}

TrafficLightState TrafficLightState::setEgoState(const EgoState& ego_state) {
    _ego_state = ego_state;
    return *this;
}

const EgoState& TrafficLightState::getEgoState() const {
    return _ego_state;
} 

TrafficLightState TrafficLightState::setStopResult(const TrafficRegulationStopResult &result) {
    _stop_result = result;
    return *this;
}

const TrafficRegulationStopResult& TrafficLightState::getStopResult() const {
    return _stop_result;
}

void TrafficLightState::updateStopResult() {
    if (_ego_state.getPathResult().hasPath() && _action == TrafficLightStateAction::BLOCKING) {
        _stop_result.updateDistance(_ego_state.getPathResult().getPath().get().front());
    }
}

