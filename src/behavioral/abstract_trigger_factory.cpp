#include "pp_env_model/behavioral/abstract_trigger_factory.hpp"

// Need these in cpp file to prevent circular include in hpp files
#include "pp_env_model/behavioral/traffic_light_state.hpp"
#include "pp_env_model/behavioral/traffic_light_trigger.hpp"
#include "pp_env_model/behavioral/ego_state.hpp"
#include "pp_env_model/behavioral/ego_trigger.hpp"
#include "pp_env_model/behavioral/stop_sign_trigger.hpp"
#include "pp_env_model/behavioral/stop_sign_state.hpp"
#include "pp_env_model/behavioral/pedestrian_trigger.hpp"
#include "pp_env_model/behavioral/pedestrian_state.hpp"
#include "pp_env_model/behavioral/obstacle_state.hpp"
#include "pp_env_model/behavioral/obstacle_trigger.hpp"
#include "pp_env_model/behavioral/global_command_state.hpp"
#include "pp_env_model/behavioral/global_command_trigger.hpp"

using namespace path_planning::behavioral;

template<typename Trigger, typename State, typename TriggerID>
boost::optional<std::shared_ptr<Trigger>> AbstractTriggerFactory<Trigger, State, TriggerID>::tryCreateTrigger(TriggerID id, State state) const {
    std::shared_ptr<Trigger> trigToTry = enumToType(id, state);
    if (trigToTry->test()) {
        return trigToTry;
    }
    return boost::none;
}

template class AbstractTriggerFactory<TrafficLightTrigger, TrafficLightState, TrafficLightTriggerID>;
template class AbstractTriggerFactory<EgoTrigger, EgoState, EgoTriggerID>;
template class AbstractTriggerFactory<StopSignTrigger, StopSignState, StopSignTriggerID>;
template class AbstractTriggerFactory<PedestrianTrigger, PedestrianState, PedestrianTriggerID>;
template class AbstractTriggerFactory<ObstacleTrigger, ObstacleState, ObstacleTriggerID>;
template class AbstractTriggerFactory<GlobalCommandTrigger, GlobalCommandState, GlobalCommandTriggerID>;
