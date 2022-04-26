#include <lanelet2_core/LaneletMap.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <utility>

#include "common_msgs/EgoLocalizationMap.h"
#include "common_msgs/HDMLane.h"
#include "common_msgs/HDMLaneLine.h"
#include "lanelet2_extension/utility/message_conversion.h"
#include "path_planning_msgs/Lanelet2MapService.h"
#include "tf/transform_datatypes.h"

using namespace lanelet;
using namespace common_msgs;

lanelet::LaneletMapPtr lanelet_map = std::make_shared<lanelet::LaneletMap>();

bool get_points(EgoLocalizationMap::Request &req,
                EgoLocalizationMap::Response &res) {
  auto line_string_to_hdm_line = [](ConstLineString2d ls) -> HDMLaneLine {
    HDMLaneLine lane_line;
    for (const auto &p : ls) {
      geometry_msgs::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = 0;
      lane_line.polyline.push_back(gp);
    }
    return lane_line;
  };
  tf::Quaternion quat;
  tf::quaternionMsgToTF(req.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO_STREAM("Received Query Point x: " << req.pose.position.x
                                             << " y: " << req.pose.position.y
                                             << " and yaw: " << yaw);
  double bottom_left_x = req.pose.position.x - req.width * sin(yaw);
  double bottom_left_y = req.pose.position.y - req.width * cos(yaw);
  double top_right_x =
      req.pose.position.x + req.width * sin(yaw) + req.length * cos(yaw);
  double top_right_y =
      req.pose.position.y + req.width * cos(yaw) + req.length * sin(yaw);
  Point2d bottom_left{utils::getId(), std::min(bottom_left_x, top_right_x),
                      std::min(bottom_left_y, top_right_y)};

  Point2d top_right{utils::getId(), std::max(bottom_left_x, top_right_x),
                    std::max(bottom_left_y, top_right_y)};

  BoundingBox2d search_bb = BoundingBox2d(bottom_left, top_right);

  ROS_INFO_STREAM("Searching for lanelets in bounding box with min "
                  << bottom_left << " and max " << top_right);

  LineStrings3d inRegion = lanelet_map->lineStringLayer.search(search_bb);

  for (const auto &ls : inRegion) {
    res.lane_lines.push_back(line_string_to_hdm_line(utils::to2D(ls)));
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ego_localization_map_service");
  ros::NodeHandle n;

  // *** Initialize Lanelet Map ***
  auto lanelet2_map_service =
      n.serviceClient<path_planning_msgs::Lanelet2MapService>(
          "/lanelet2_map_service");
  path_planning_msgs::Lanelet2MapService map_call;
  lanelet2_map_service.call(map_call);
  lanelet::utils::conversion::fromBinMsg(map_call.response.map_bin,
                                         lanelet_map);

  ros::ServiceServer map_service =
      n.advertiseService("/ego_localization_map", get_points);
  ros::spin();

  return 0;
}
