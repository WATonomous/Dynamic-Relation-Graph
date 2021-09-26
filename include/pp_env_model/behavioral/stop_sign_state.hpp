#pragma once

#include "abstract_state.hpp"
#include "pp_common/stop_sign.hpp"
#include "stop_result.hpp"
#include "pp_common/stopline.hpp"
#include "ros/ros.h"
#include "ego_state.hpp"

namespace path_planning { namespace behavioral {

    enum class StopSignStateAction {NON_BLOCKING, BLOCKING, ACTIVE_BLOCKING, INACTIVE};

    class StopSignState : public AbstractState<StopSignStateAction> {
        public:

            StopSignState(StopSign sign, StopSignStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            StopSignState setSign(const StopSign& sign);
            const StopSign& getSign() const;

            StopSignState setEgoState(const EgoState& ego_state);
            const EgoState& getEgoState() const;

            StopSignState setStopResult(const TrafficRegulationStopResult &result);
            const TrafficRegulationStopResult& getStopResult() const;

            StopSignState setActivatedTime(ros::Time time);
            ros::Time getActivatedTime() const;

            void updateStopResult();

        private:
            StopSign _sign;
            TrafficRegulationStopResult _stop_result;
            ros::Time _activated_time;
            EgoState _ego_state;
    };

} }