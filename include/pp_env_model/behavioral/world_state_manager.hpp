#pragma one

#include <vector>
#include <map>
#include <memory>
#include "pp_common/line_segment.hpp"
#include "pp_env_model/behavioral/traffic_light_state_machine.hpp"
#include "pp_common/stopline.hpp"
#include "pp_common/polyline.hpp"
#include "pp_common/environment.hpp"
#include "pp_common/global_command.hpp"
#include "pp_common/vehicle_state.hpp"
#include "pp_env_model/behavioral/stop_result.hpp"
#include "pp_env_model/behavioral/stop_sign_state_machine.hpp"
#include "pp_env_model/behavioral/pedestrian_state_machine.hpp"
#include "pp_env_model/behavioral/ego_state.hpp"
#include "pp_common/lane.hpp"
#include "pp_env_model/behavioral/obstacle_state_machine.hpp"
#include <unordered_set>
#include <utility>

namespace path_planning { namespace behavioral {

    class WorldStateManager {

        public:

            void updateEnvironment(const path_planning::Environment &env);

            void updateGlobalCommandQueue(path_planning::GlobalCommandQueue cmds);

            void updateEgoState(const EgoState& ego_state);

            void cycleMachines();

            std::vector<path_planning::StopLine> getStopLines() const;

            StopResult getStopResult() const;

        private:
            std::map<int, std::shared_ptr<TrafficLightStateMachine>> _traffic_lights;
            std::map<int, std::shared_ptr<StopSignStateMachine>> _stop_signs;
            std::map<int, std::shared_ptr<PedestrianStateMachine>> _pedestrians;
            std::map<int, std::shared_ptr<ObstacleStateMachine>> _obstacles;
    };

}}