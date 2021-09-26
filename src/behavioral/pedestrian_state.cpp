#include "pp_env_model/behavioral/pedestrian_state.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

PedestrianState::PedestrianState(Pedestrian ped_obj, PedestrianStateAction action):
    AbstractState(action), _ped_obj{ped_obj}, _ego_state{EgoStateAction::WAITING_FOR_COMMANDS} {}

std::string PedestrianState::serializeAction() const {
    switch(_action) {
        case PedestrianStateAction::IGNORED:
            return "IGNORED";
        case PedestrianStateAction::NON_ACTIVE:
            return "NON_ACTIVE";
        case PedestrianStateAction::ACTIVE:
            return "ACTIVE";
        case PedestrianStateAction::PAUSED:
            return "PAUSED";
        default:
            return "UNKNOWN, update PedestrianState::serializeAction()";
    }
}

std::string PedestrianState::serialize() const {
    return "";
}

PedestrianState PedestrianState::setPedestrianObj(const Pedestrian& ped_obj) {
    _ped_obj = ped_obj;
    return *this;
}

const Pedestrian& PedestrianState::getPedestrianObj() const {
    return _ped_obj;
}

PedestrianState PedestrianState::setEgoState(const EgoState& ego_state) {
    _ego_state = ego_state;
    return *this;
}

const EgoState& PedestrianState::getEgoState() const {
    return _ego_state;
}

PedestrianState PedestrianState::setStopResult(const DynamicObjectStopResult &result) {
    _stop_result = result;
    return *this;
}

const DynamicObjectStopResult& PedestrianState::getStopResult() const {
    return _stop_result;
}

PedestrianState PedestrianState::setPausedTime(ros::Time time) {
    _paused_time = time;
    return *this;
}

ros::Time PedestrianState::getPausedTime() const {
    return _paused_time;
}

void PedestrianState::updateStopResult() {
    if (_ego_state.getPathResult().hasPath() && (_action == PedestrianStateAction::ACTIVE || _action == PedestrianStateAction::PAUSED)) {
        _stop_result.updateDistance(_ego_state.getPathResult().getFrenetReferenceLine().get(), _ego_state.getPose(), _ped_obj);
    }
}