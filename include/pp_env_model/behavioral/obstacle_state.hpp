#pragma once

#include "abstract_state.hpp"
#include "pp_common/obstacle.hpp"
#include "stop_result.hpp"
#include "ego_state.hpp"

namespace path_planning { namespace behavioral {

    enum class ObstacleStateAction {IGNORED, MOVING, STOPPED};

    class ObstacleState : public AbstractState<ObstacleStateAction> {
        public:

            ObstacleState(Obstacle obj, ObstacleStateAction action);

            std::string serializeAction() const override;
            std::string serialize() const override;

            ObstacleState setObstacle(const Obstacle& obj);
            const Obstacle& getObstacle() const;

            ObstacleState setEgoState(const EgoState& ego_state);
            const EgoState& getEgoState() const;

            ObstacleState setVehiclePose(const VehicleState &pose);
            const VehicleState& getVehiclePose() const;

            ObstacleState setStopResult(const DynamicObjectStopResult &result);
            const DynamicObjectStopResult& getStopResult() const;

            void updateStopResult();

        private:
            Obstacle _obj;
            DynamicObjectStopResult _stop_result;
            EgoState _ego_state;
    };

} }