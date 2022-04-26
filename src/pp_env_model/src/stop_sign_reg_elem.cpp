#include "pp_env_model/stop_sign_reg_elem.hpp"

#include <ros/ros.h>

namespace path_planning {
namespace env_model {

lanelet::ConstLineString3d StopSignRegElem::getStopLine() const {
  return getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine)
      .front();
}

void StopSignRegElem::pushEgoOdom(nav_msgs::Odometry &odom) {
  bool full_hist = false;
  while (!ego_hist_.empty() &&
         (odom.header.stamp - ego_hist_.back().header.stamp).toSec() >
             kWaitTime) {
    ego_hist_.pop_back();
    full_hist = true;
  }
  ego_hist_.insert(ego_hist_.begin(), odom);
  if (!full_hist) {
    return;
  }
  for (const auto &odom : ego_hist_) {
    if (odom.twist.twist.linear.x > kStopVel ||
        odom.twist.twist.linear.y > kStopVel ||
        lanelet::geometry::distance2d(
            utils::geomToPoint(odom.pose.pose.position), getStopLine()) >
            kStopDist) {
      ROS_DEBUG_STREAM("odom.twist.twist.linear.x "
                       << odom.twist.twist.linear.x);
      ROS_DEBUG_STREAM("odom.twist.twist.linear.x "
                       << odom.twist.twist.linear.y);
      ROS_DEBUG_STREAM(
          "distance2d " << lanelet::geometry::distance2d(
              utils::geomToPoint(odom.pose.pose.position), getStopLine()));
      return;
    }
  }
  done_ = true;
}

bool StopSignRegElem::shouldStop() const { return !done_; }

StopSignRegElem::StopSignRegElem(lanelet::Id id,
                                 lanelet::LineString3d stop_sign_loc)
    : RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)} {
  parameters().insert({lanelet::RoleNameString::RefLine, {stop_sign_loc}});
}

StopSignRegElem::StopSignRegElem(const lanelet::RegulatoryElementDataPtr &data)
    : RegulatoryElement(data) {}

static lanelet::RegisterRegulatoryElement<StopSignRegElem> regStop;
constexpr char StopSignRegElem::RuleName[];

}  // namespace env_model
}  // namespace path_planning
