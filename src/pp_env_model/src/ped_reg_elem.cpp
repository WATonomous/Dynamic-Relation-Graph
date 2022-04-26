#include "pp_env_model/ped_reg_elem.hpp"

namespace path_planning {
namespace env_model {

// returns the predicted path of the pedestrian
lanelet::ConstLineString3d PedRegElem::getPredictedStates() const {
  return getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine)
      .front();
}

bool PedRegElem::isTrackDead() const {
  return (ros::Time::now() - _last_seen_time).toSec() > 5;
}

void PedRegElem::updatePredictedStates(lanelet::LineString3d predicted_states) {
  parameters()[lanelet::RoleNameString::RefLine] = {predicted_states};
  _last_seen_time = ros::Time::now();
}

PedRegElem::PedRegElem(lanelet::Id id, lanelet::LineString3d predictedStates)
    : RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)},
      _last_seen_time{ros::Time::now()} {
  parameters().insert({lanelet::RoleNameString::RefLine, {predictedStates}});
}

PedRegElem::PedRegElem(const lanelet::RegulatoryElementDataPtr& data)
    : RegulatoryElement(data) {}

static lanelet::RegisterRegulatoryElement<PedRegElem> regPed;
constexpr char PedRegElem::RuleName[];

}  // namespace env_model
}  // namespace path_planning
