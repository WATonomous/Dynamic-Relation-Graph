#pragma once

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <ros/time.h>

namespace path_planning {
namespace env_model {

class PedRegElem : public lanelet::RegulatoryElement {
 public:
  // lanelet2 looks for this string when matching the subtype of a regulatory
  // element to the respective type
  static constexpr char RuleName[] = "pedestrian";

  // returns the predicted path of the pedestrian
  lanelet::ConstLineString3d getPredictedStates() const;

  bool isTrackDead() const;

  void updatePredictedStates(lanelet::LineString3d predicted_states);

 private:
  PedRegElem(lanelet::Id id, lanelet::LineString3d predictedStates);

  ros::Time _last_seen_time;

  friend class lanelet::RegisterRegulatoryElement<PedRegElem>;
  explicit PedRegElem(const lanelet::RegulatoryElementDataPtr& data);
};

}  // namespace env_model
}  // namespace path_planning
