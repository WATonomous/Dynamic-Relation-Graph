#pragma once

#include "abstract_state.hpp"
#include "pp_common/pedestrian.hpp"
#include "stop_result.hpp"
#include "ego_state.hpp"

namespace path_planning { namespace behavioral {

    enum class PedestrianStateAction {IGNORED, NON_ACTIVE, ACTIVE, PAUSED};

    class PedestrianState : public AbstractState<PedestrianStateAction> {
        public:

            PedestrianState(Pedestrian ped_obj, PedestrianStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            PedestrianState setPedestrianObj(const Pedestrian& ped_obj);
            const Pedestrian& getPedestrianObj() const;

            PedestrianState setEgoState(const EgoState& ego_state);
            const EgoState& getEgoState() const;

            PedestrianState setStopResult(const DynamicObjectStopResult &result);
            const DynamicObjectStopResult& getStopResult() const;

            PedestrianState setPausedTime(ros::Time time);
            ros::Time getPausedTime() const;

            void updateStopResult();

        private:
            Pedestrian _ped_obj;
            DynamicObjectStopResult _stop_result;
            ros::Time _paused_time;
            EgoState _ego_state;
    };

} }