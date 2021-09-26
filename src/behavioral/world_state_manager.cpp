#include "pp_env_model/behavioral/world_state_manager.hpp"

using namespace path_planning::behavioral;

void WorldStateManager::updateEnvironment(const path_planning::Environment &env) {
    for (const std::shared_ptr<path_planning::TrafficLight> &light : env.traffic_lights) {
        if (_traffic_lights.find(light->id) != _traffic_lights.end()) {
            _traffic_lights[light->id]->setState(_traffic_lights[light->id]->getState().setLightStatus(*light));
        }
        else {
            _traffic_lights.insert(std::make_pair(light->id, std::make_shared<TrafficLightStateMachine>(*light)));
        }
    }

    for (const std::shared_ptr<path_planning::TrafficSign> &sign : env.traffic_signs) {
        if(const std::shared_ptr<path_planning::StopSign> & stop_sign = std::dynamic_pointer_cast<StopSign>(sign)) {
            if (_stop_signs.find(stop_sign->id) != _stop_signs.end()) {
                _stop_signs[stop_sign->id]->setState(_stop_signs[stop_sign->id]->getState().setSign(*stop_sign));
            }
            else {
                _stop_signs.insert(std::make_pair(stop_sign->id, std::make_shared<StopSignStateMachine>(*stop_sign)));
            }
        }
    }

    for (const std::shared_ptr<OccupiableObject> &obj : env.obstacles) {
        if (const std::shared_ptr<path_planning::Pedestrian> & ped_obj = std::dynamic_pointer_cast<Pedestrian>(obj)) {
            if (_pedestrians.find(ped_obj->id) != _pedestrians.end()) {
                _pedestrians[ped_obj->id]->setState(_pedestrians[ped_obj->id]->getState().setPedestrianObj(*ped_obj));
            }
            else {
                _pedestrians.insert(std::make_pair(ped_obj->id, std::make_shared<PedestrianStateMachine>(*ped_obj)));
            }
        }
        else if (const std::shared_ptr<path_planning::Obstacle> & obs = std::dynamic_pointer_cast<Obstacle>(obj)) {
            if (_obstacles.find(obs->id) != _obstacles.end()) {
                _obstacles[obs->id]->setState(_obstacles[obs->id]->getState().setObstacle(*obs));
            }
            else {
                _obstacles.insert(std::make_pair(obs->id, std::make_shared<ObstacleStateMachine>(*obs)));
            }
        }
    }

    // TODO: Remove lights not in env from world?
}

void WorldStateManager::updateGlobalCommandQueue(path_planning::GlobalCommandQueue cmds) {
    for (auto const& traffic_light : _traffic_lights) {
        traffic_light.second->setState(traffic_light.second->getState().setGlobalCommandQueue(cmds));
    }
}

void WorldStateManager::cycleMachines() {
    for (auto const& traffic_light : _traffic_lights) {
        traffic_light.second->cycle();
    }

    for (auto const& stop_sign : _stop_signs) {
        stop_sign.second->cycle();
    }

    for (auto const& ped : _pedestrians) {
        ped.second->cycle();
    }

    for (auto const& obj : _obstacles) {
        obj.second->cycle();
    }
}

StopResult WorldStateManager::getStopResult() const {
    double min_stop_dist = DBL_MAX;
    StopResult ret;
    for (auto const& traffic_light : _traffic_lights) {
        if (traffic_light.second->getState().getAction() == TrafficLightStateAction::BLOCKING 
            && traffic_light.second->getState().getStopResult().hasStop()) {
            if (traffic_light.second->getState().getStopResult().getDistance() < min_stop_dist) {
                ret = traffic_light.second->getState().getStopResult();
                min_stop_dist = traffic_light.second->getState().getStopResult().getDistance();
            }
        }
    }
    for (auto const& stop_sign : _stop_signs) {
        if ((stop_sign.second->getState().getAction() == StopSignStateAction::BLOCKING 
            || stop_sign.second->getState().getAction() == StopSignStateAction::ACTIVE_BLOCKING) 
            && stop_sign.second->getState().getStopResult().hasStop()) {
            if (stop_sign.second->getState().getStopResult().getDistance() < min_stop_dist) {
                ret = stop_sign.second->getState().getStopResult();
                min_stop_dist = stop_sign.second->getState().getStopResult().getDistance();
            }
        }
    }
    for (auto const& ped : _pedestrians) {
        if ((ped.second->getState().getAction() == PedestrianStateAction::ACTIVE 
            || ped.second->getState().getAction() == PedestrianStateAction::PAUSED) 
            && ped.second->getState().getStopResult().hasStop()) {
            if (ped.second->getState().getStopResult().getDistance() < min_stop_dist) {
                ret = ped.second->getState().getStopResult();
                min_stop_dist = ped.second->getState().getStopResult().getDistance();
            }
        }
    }
    for (auto const& obj : _obstacles) {
        if (obj.second->getState().getAction() == ObstacleStateAction::MOVING 
            && obj.second->getState().getStopResult().hasStop()) {
            if (obj.second->getState().getStopResult().getDistance() < min_stop_dist) {
                ret = obj.second->getState().getStopResult();
                min_stop_dist = obj.second->getState().getStopResult().getDistance();
            }
        }
    }
    return ret;
}

void WorldStateManager::updateEgoState(const EgoState& ego_state) {
    for (auto const& traffic_light : _traffic_lights) {
        traffic_light.second->setState(traffic_light.second->getState().setEgoState(ego_state));
    }
    for (auto const& stop_sign : _stop_signs) {
        stop_sign.second->setState(stop_sign.second->getState().setEgoState(ego_state));
    }
    for (auto const& ped : _pedestrians) {
        ped.second->setState(ped.second->getState().setEgoState(ego_state));
    }
    for (auto const& obj : _obstacles) {
        obj.second->setState(obj.second->getState().setEgoState(ego_state));
    }
}

