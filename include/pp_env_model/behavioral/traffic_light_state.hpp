#pragma once

#include "ros/ros.h"
#include "pp_common/line_segment.hpp"
#include "pp_common/traffic_light.hpp"
#include "pp_common/global_command.hpp"
#include "pp_common/stopline.hpp"
#include "abstract_state.hpp"
#include "pp_common/polyline.hpp"
#include "pp_common/vehicle_state.hpp"
#include "ego_state.hpp"
#include "pp_common/lane.hpp"
#include "stop_result.hpp"

namespace path_planning { namespace behavioral {

    enum class TrafficLightStateAction {NON_BLOCKING, BLOCKING};

    class TrafficLightState : public AbstractState<TrafficLightStateAction> {
        public:

            TrafficLightState(TrafficLight light, TrafficLightStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            TrafficLightState setLightStatus(TrafficLight light);
            TrafficLight getLightStatus() const;

            TrafficLightState setGlobalCommandQueue(GlobalCommandQueue cmds);
            GlobalCommandQueue getGlobalCommandQueue() const;

            TrafficLightState setEgoState(const EgoState &ego_state);
            const EgoState& getEgoState() const;

            TrafficLightState setStopResult(const TrafficRegulationStopResult &result);
            const TrafficRegulationStopResult& getStopResult() const;

            void updateStopResult();

        private:
            TrafficLight _light;
            GlobalCommandQueue _globalCommandQueue;
            TrafficRegulationStopResult _stop_result;
            EgoState _ego_state;
    };

}}