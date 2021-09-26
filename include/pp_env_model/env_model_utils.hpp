#pragma once

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/LaneletPath.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <unordered_set>

#include "common_msgs/TrackedObstacleList.h"
#include "common_msgs/TrafficLight.h"
#include "common_msgs/TrafficSign.h"
#include "lanelet2_extension/visualization/visualization.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pp_env_model/ped_reg_elem.hpp"
#include "pp_env_model/stop_sign_reg_elem.hpp"

namespace path_planning {
namespace env_model {

using DistLanelet = std::pair<double, lanelet::Lanelet>;
using DistLaneletList = std::vector<DistLanelet>;
using ConstLaneletSet = std::unordered_set<lanelet::ConstLanelet>;
using LaneletSet = std::unordered_set<lanelet::Lanelet>;
using Destination =
    std::pair<std::vector<lanelet::ConstLanelet>, lanelet::Point2d>;
using DestinationList = std::vector<Destination>;
struct ProjInfo {
  lanelet::BasicPoint3d p;
  double dist;
};

namespace utils {

bool signIsRelevantToLanelet(const lanelet::Lanelet &ll,
                             const common_msgs::TrafficSign &sign);

std::vector<lanelet::Id> extractIds(lanelet::ConstLanelets lls);
std::vector<lanelet::Id> extractIds(DistLaneletList lls);

// Helper function for traffic_signs_callback: Create RegElem to relate the
// Traffic Sign and the lanelet
lanelet::RegulatoryElementPtr buildTrafficSignRegElem(
    const common_msgs::TrafficSign &signObj);

lanelet::Lanelets pedestrianConflicts(lanelet::LaneletLayer &layer,
                                      const common_msgs::TrackedObstacle &ped);
lanelet::LineString3d obsStateToLs(
    const std::vector<common_msgs::TrackedObstacleState> &states);
double checkStop(const lanelet::ConstLanelet &ll);
std::vector<geometry_msgs::Point> lsToGeom(
    const lanelet::BasicLineString2d &ref_ls);
lanelet::Point3d geomToPoint(const geometry_msgs::Point p,
                             bool allocate_id = false);
geometry_msgs::Point pointToGeom(const lanelet::ConstPoint3d &p);
geometry_msgs::Point pointToGeom(const lanelet::BasicPoint3d &p);
geometry_msgs::Point pointToGeom(const lanelet::ConstPoint2d &p);
geometry_msgs::Point pointToGeom(const lanelet::BasicPoint2d &p);
std::vector<geometry_msgs::Point> postProessRef(
    std::vector<geometry_msgs::Point> &ref, double back_dist);
visualization_msgs::MarkerArray createRefLineMarker(
    const std::vector<geometry_msgs::Point> &);
visualization_msgs::MarkerArray regElemMarkers(const lanelet::Lanelet &ll);
visualization_msgs::MarkerArray mapMarkers(lanelet::LaneletMapPtr);
visualization_msgs::MarkerArray createRouteMarker(
    const lanelet::routing::LaneletPath &);
visualization_msgs::MarkerArray createDestinationMarker(
    const DestinationList &);
visualization_msgs::MarkerArray deleteAllMarker();
lanelet::Optional<geometry_msgs::Point> occupancyToPoint(
    int index, const nav_msgs::OccupancyGrid &occupancy_grid);
int pointToOccupancy(const geometry_msgs::TransformStamped &map_to_base_link_tf,
                     geometry_msgs::Point p,
                     const nav_msgs::OccupancyGrid &occupancy_grid);

}  // namespace utils
}  // namespace env_model
}  // namespace path_planning
