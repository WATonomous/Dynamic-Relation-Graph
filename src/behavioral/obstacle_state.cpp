#include "pp_env_model/behavioral/obstacle_state.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

ObstacleState::ObstacleState(Obstacle obj, ObstacleStateAction action):
    AbstractState(action), _obj{obj}, _ego_state{EgoStateAction::WAITING_FOR_COMMANDS} {}

std::string ObstacleState::serializeAction() const {
    switch(_action) {
        case ObstacleStateAction::IGNORED:
            return "IGNORED";
        case ObstacleStateAction::MOVING:
            return "MOVING";
        case ObstacleStateAction::STOPPED:
            return "STOPPED";
        default:
            return "UNKNOWN, update ObstacleState::serializeAction()";
    }
}

std::string ObstacleState::serialize() const {
    return "";
}

ObstacleState ObstacleState::setObstacle(const Obstacle& obj) {
    _obj = obj;
    return *this;
}

const Obstacle& ObstacleState::getObstacle() const {
    return _obj;
}

ObstacleState ObstacleState::setEgoState(const EgoState& ego_state) {
    _ego_state = ego_state;
    return *this;
}

const EgoState& ObstacleState::getEgoState() const {
    return _ego_state;
}

ObstacleState ObstacleState::setStopResult(const DynamicObjectStopResult &result) {
    _stop_result = result;
    return *this;
}

const DynamicObjectStopResult& ObstacleState::getStopResult() const {
    return _stop_result;
}

void ObstacleState::updateStopResult() {
    if (_ego_state.getPathResult().hasPath() && _action == ObstacleStateAction::MOVING) {
        _stop_result.updateDistance(_ego_state.getPathResult().getFrenetReferenceLine().get(), _ego_state.getPose(), _obj);
    }
}