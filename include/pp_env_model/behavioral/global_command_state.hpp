#pragma once

#include "ros/ros.h"
#include "pp_common/line_segment.hpp"
#include "pp_common/lane.hpp"
#include "pp_common/global_command.hpp"
#include "pp_common/stopline.hpp"
#include "pp_env_model/behavioral/abstract_state.hpp"
#include "pp_common/polyline.hpp"
#include "pp_common/vehicle_state.hpp"
#include "pp_env_model/behavioral/stop_result.hpp"
#include "pp_env_model/behavioral/ego_state.hpp"

using boost::optional;
using path_planning::Lane;

namespace path_planning { namespace behavioral {

    enum class GlobalCommandStateAction {TURNING_RIGHT, TURNING_LEFT, HEADING_STRAIGHT, STOPPING, WAITING_FOR_GLOBAL_COMMAND, DONE};

    class GlobalCommandState : public AbstractState<GlobalCommandStateAction> {
        public:

            GlobalCommandState(GlobalCommandStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            GlobalCommandState setGlobalCommandQueue(const GlobalCommandQueue &cmds);
            GlobalCommandQueue getGlobalCommandQueue() const;

            GlobalCommandState setEgoState(const EgoState &ego_state);
            EgoState getEgoState() const;

        private:
            GlobalCommandQueue _global_cmd_queue;
            EgoState _ego_state;
    };

}}
