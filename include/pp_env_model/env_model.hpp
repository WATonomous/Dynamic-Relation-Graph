#pragma once

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <unordered_set>

#include "common_msgs/TrackedObstacleList.h"
#include "common_msgs/TrafficLightList.h"
#include "common_msgs/TrafficSignList.h"
#include "path_planning_msgs/ReferenceLine.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "pp_env_model/env_model_utils.hpp"
#include "pp_env_model/ped_reg_elem.hpp"
#include "pp_env_model/routing.hpp"
#include "ros/ros.h"

namespace path_planning {
namespace env_model {

class EnvModel {
 public:
  Router router;
  ros::Publisher route_viz_pub;

  EnvModel(lanelet::LaneletMapPtr laneletMap, ros::NodeHandle &nh);
  lanelet::LaneletMapPtr map();
  void trafficSignCallback(
      const common_msgs::TrafficSignList &traffic_sign_list);
  void trafficLightDetectionCallback(
      const common_msgs::TrafficLight &tl_status);
  void updateStopSigns(nav_msgs::Odometry ego_odom);
  void trackedObstacleCallback(
      const common_msgs::TrackedObstacleList &tracked_obs_list);
  void occGridCallback(const nav_msgs::OccupancyGrid &occupancy_grid);
  void pruneDeadTracks(const ros::TimerEvent &);
  lanelet::Optional<common_msgs::TrafficLightList> nextTrafficLight();
  path_planning_msgs::ReferenceLine localRef(
      const lanelet::Point3d &ego_location);
  bool route();

 private:
  std::string kMapFrameName = "odom";
  std::string kVechielFrameName = "base_link";
  int kMaxSearchDepthForIntersectionLlBFS = 2;
  double kStopSignSearchRad = 20;
  double kLocalLookAhead = 60;
  double kLocalLookBack = 15;

  lanelet::LaneletMapPtr lanelet_map_;
  std::unordered_map<uint32_t, lanelet::Id> tracker_map_id_;
  tf2_ros::Buffer tf_buffer_;
  ros::Publisher filtered_occ_pub_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  void stopSignAssociation(const common_msgs::TrafficSign &traffic_sign);
  void intersectionSignsAssociation(
      std::vector<common_msgs::TrafficSign> signs);
  DistLaneletList getEgoLanelets();
  double longitudinal_distance(
      lanelet::BasicPoint2d ego, lanelet::Lanelet &lanelet,
      lanelet::BasicSegment2d obstacle_segment,
      std::vector<lanelet::BasicPoint2d> obstacle_points, bool backwards);
  double distanceFromStartToClosestPoint(
      lanelet::BasicPoint2d start,
      std::vector<lanelet::BasicPoint2d> obstacle_points);

  void lateral_shift(const nav_msgs::OccupancyGrid &occupancy_grid);
};

}  // namespace env_model
}  // namespace path_planning
