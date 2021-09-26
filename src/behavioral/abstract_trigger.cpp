#include "pp_env_model/behavioral/abstract_trigger.hpp"

// Need these in cpp file to prevent circular include in hpp files
#include "pp_env_model/behavioral/traffic_light_state.hpp"
#include "pp_env_model/behavioral/traffic_light_trigger.hpp"
#include "pp_env_model/behavioral/ego_state.hpp"
#include "pp_env_model/behavioral/ego_trigger.hpp"
#include "pp_env_model/behavioral/stop_sign_state.hpp"
#include "pp_env_model/behavioral/stop_sign_trigger.hpp"
#include "pp_env_model/behavioral/pedestrian_state.hpp"
#include "pp_env_model/behavioral/pedestrian_trigger.hpp"
#include "pp_env_model/behavioral/obstacle_state.hpp"
#include "pp_env_model/behavioral/obstacle_trigger.hpp"
#include "pp_env_model/behavioral/global_command_trigger.hpp"

using namespace path_planning::behavioral;

template<typename TriggerID, typename State>
AbstractTrigger<TriggerID, State>::AbstractTrigger(TriggerID id, State state): _state{state}, _id{id} {} 

template<typename TriggerID, typename State>
TriggerID AbstractTrigger<TriggerID, State>::getID() const {
    return _id;
}

template class AbstractTrigger<TrafficLightTriggerID, TrafficLightState>;
template class AbstractTrigger<EgoTriggerID, EgoState>;
template class AbstractTrigger<StopSignTriggerID, StopSignState>;
template class AbstractTrigger<PedestrianTriggerID, PedestrianState>;
template class AbstractTrigger<ObstacleTriggerID, ObstacleState>;
template class AbstractTrigger<GlobalCommandTriggerID, GlobalCommandState>;
