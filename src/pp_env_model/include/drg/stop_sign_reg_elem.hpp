#pragma once
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include "nav_msgs/Odometry.h"
#include "pp_env_model/env_model_utils.hpp"

namespace path_planning {
namespace env_model {

class StopSignRegElem : public lanelet::RegulatoryElement {
 public:
  // Lanelet2 looks for this string when matching the subtype of a regulatory
  // element to the respective type.
  static constexpr char RuleName[] = "stop_sign";

  // Returns the line where we are suppose to stop.
  lanelet::ConstLineString3d getStopLine() const;
  void pushEgoOdom(nav_msgs::Odometry&);
  bool shouldStop() const;

 private:
  double kStopVel = 0.5;
  double kStopDist = 10.0;
  double kWaitTime = 5.0;
  bool done_ = false;
  std::vector<nav_msgs::Odometry> ego_hist_;

  StopSignRegElem(lanelet::Id id, lanelet::LineString3d stop_sign_loc);

  // The following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element.
  friend class lanelet::RegisterRegulatoryElement<StopSignRegElem>;
  explicit StopSignRegElem(const lanelet::RegulatoryElementDataPtr& data);
};

}  // namespace env_model
}  // namespace path_planning
