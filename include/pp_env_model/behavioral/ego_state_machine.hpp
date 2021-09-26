#pragma once

#include <vector>
#include <memory>
#include <ros/ros.h>
#include <embedded_msgs/LockLightingRequest.h>
#include "pp_env_model/behavioral/ego_state.hpp"
#include "pp_env_model/behavioral/ego_trigger.hpp"
#include "pp_env_model/behavioral/abstract_transition_graph.hpp"
#include "pp_env_model/behavioral/abstract_state_machine.hpp"
#include "pp_env_model/behavioral/global_interface.hpp"
#include "pp_common/global_command.hpp"

namespace path_planning { namespace behavioral {

    class EgoStateMachine : public AbstractStateMachine<EgoState, EgoTrigger, EgoStateAction, EgoTriggerID> {
        public:
            EgoStateMachine(
                const ros::Publisher &light_lock_publisher
            );

            void cycle() override;
            std::string getName() const override;
            
        private:
            AbstractTransitionGraph<EgoStateAction, EgoTriggerID> createTransitionGraph() override;
            const ros::Publisher &light_lock_publisher;

            bool transition(const EgoTrigger &trigger) override;
    };

}}