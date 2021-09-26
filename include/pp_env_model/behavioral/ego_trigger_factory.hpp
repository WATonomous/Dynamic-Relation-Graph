#pragma once

#include "pp_env_model/behavioral/ego_trigger.hpp"
#include "pp_env_model/behavioral/ego_state.hpp"
#include "pp_env_model/behavioral/abstract_trigger_factory.hpp"

namespace path_planning { namespace behavioral {

    class EgoTriggerFactory : public AbstractTriggerFactory<EgoTrigger, EgoState, EgoTriggerID> {
        private:
            std::shared_ptr<EgoTrigger> enumToType(EgoTriggerID id, EgoState state) const override;
    };

} }