#include "pp_env_model/env_model_utils.hpp"

namespace path_planning {
namespace env_model {
namespace utils {

/**
 * This function takes in a point in the map frame and returns the index of the
 * occupancy grid in which the point lies in.
 * Takes in TransfromStamped msg to improve performance
 * If the point does not lie in the occupancy grid, the function returns -1
 */
int pointToOccupancy(const geometry_msgs::TransformStamped &map_to_base_link_tf,
                     geometry_msgs::Point p,
                     const nav_msgs::OccupancyGrid &occupancy_grid) {
  geometry_msgs::Point base_link_p;
  geometry_msgs::Point base_link_origin;

  try {
    tf2::doTransform(p, base_link_p, map_to_base_link_tf);
    tf2::doTransform(occupancy_grid.info.origin.position, base_link_origin,
                     map_to_base_link_tf);
  } catch (tf2::TransformException &ex) {
    ROS_WARN(
        "A map to base link transform in the pointToOccupancy function failed: "
        "%s",
        ex.what());
    return -1;
  }
  int x = abs(floor((base_link_p.x - base_link_origin.x) /
                    occupancy_grid.info.resolution));
  int y = abs(floor((base_link_p.y - base_link_origin.y) /
                    occupancy_grid.info.resolution));
  // cout << "x: " << x << ", y: " << y << endl;
  int num_grid_x = occupancy_grid.info.width;   // x is longitudinal direction
  int num_grid_y = occupancy_grid.info.height;  // y is lateral direction
  // cout << "num_grid_x: " << num_grid_x << ", num_grid_y: " << num_grid_y <<
  // endl;
  return (x * num_grid_x + y >= num_grid_x * num_grid_y) ? -1
                                                         : y * num_grid_x + x;
}

/**
 * This function takes in an index of the occupancy matrix
 * and returns the centre point of the occupancy grid corresponding to that
 *index The x axis of the occupancy grid is in the longitudinal direction of the
 *car (front & back). The y axis of the occupancy grid is in the lateral
 *direction of the car (left & right) The function does not return a point if
 *any of the frame transforms fail.
 **/
lanelet::Optional<geometry_msgs::Point> occupancyToPoint(
    int index, const nav_msgs::OccupancyGrid &occupancy_grid) {
  int num_grid_x = occupancy_grid.info.width;   // x is longitudinal direction
  int num_grid_y = occupancy_grid.info.height;  // y is lateral direction
  int x = index % num_grid_x;
  int y = index / num_grid_x;
  assert(x < num_grid_x && x >= 0);
  assert(y < num_grid_y && y >= 0);

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "grid_link";
  tf.transform.translation.x = occupancy_grid.info.origin.position.x;
  tf.transform.translation.y = occupancy_grid.info.origin.position.y;
  tf.transform.translation.z = occupancy_grid.info.origin.position.z;
  tf.transform.rotation = occupancy_grid.info.origin.orientation;
  geometry_msgs::PointStamped grid_ps;
  grid_ps.header.frame_id = "grid_link";
  grid_ps.point.x =
      x * occupancy_grid.info.resolution + occupancy_grid.info.resolution / 2;
  grid_ps.point.y =
      y * occupancy_grid.info.resolution + occupancy_grid.info.resolution / 2;
  geometry_msgs::PointStamped ps;

  try {
    tf2::doTransform(grid_ps, ps, tf);
  } catch (tf2::TransformException &ex) {
    ROS_WARN(
        "A grid_link to odom transform in the occupancyToPoint function "
        "failed: "
        "%s",
        ex.what());
    return {};
  }
  geometry_msgs::Point p = ps.point;
  return p;
}

bool signIsRelevantToLanelet(const lanelet::Lanelet &ll,
                             const common_msgs::TrafficSign &sign) {
  std::string turnDirectionAttr = ll.attribute("turn_direction").value();
  std::string signType = sign.traffic_sign_type;

  if ((turnDirectionAttr == "left" &&
       (signType == common_msgs::TrafficSign::TS_TP_NO_LEFT ||
        signType == common_msgs::TrafficSign::TS_TP_RIGHT)) ||
      (turnDirectionAttr == "right" &&
       (signType == common_msgs::TrafficSign::TS_TP_NO_RIGHT ||
        signType == common_msgs::TrafficSign::TS_TP_LEFT)) ||
      (turnDirectionAttr == "straight" &&
       (signType == common_msgs::TrafficSign::TS_TP_DNE ||
        signType == common_msgs::TrafficSign::TS_TP_LEFT ||
        signType == common_msgs::TrafficSign::TS_TP_RIGHT))) {
    ROS_DEBUG_STREAM("RegElem Associated On " << turnDirectionAttr << " "
                                              << signType);
    return true;
  }

  ROS_DEBUG_STREAM("No RegElem Associated On " << turnDirectionAttr << " "
                                               << signType);
  return false;
}

std::vector<lanelet::Id> extractIds(lanelet::ConstLanelets lls) {
  std::vector<lanelet::Id> ret;
  for (const auto &ll : lls) {
    ret.push_back(ll.id());
  }
  return ret;
}

std::vector<lanelet::Id> extractIds(DistLaneletList lls) {
  std::vector<lanelet::Id> ret;
  for (const auto &ll : lls) {
    ret.push_back(ll.second.id());
  }
  return ret;
}

// Helper function for traffic_signs_callback: Create RegElem to relate the
// Traffic Sign and the lanelet
lanelet::RegulatoryElementPtr buildTrafficSignRegElem(
    const common_msgs::TrafficSign &signObj) {
  // Build up the object needed inorder to declare TrafficSign RegElem object
  lanelet::Point3d p{lanelet::utils::getId(), signObj.pose.position.x,
                     signObj.pose.position.y, signObj.pose.position.z};
  lanelet::LineString3d signLocation(lanelet::utils::getId(), {p});

  lanelet::TrafficSignsWithType signTypeObject;
  signTypeObject.trafficSigns.push_back(
      signLocation);  // Location of sign stored in RegElem
  signTypeObject.type = signObj.traffic_sign_type;  // Type of RegElem

  return lanelet::TrafficSign::make(lanelet::utils::getId(), {},
                                    {signTypeObject});
}

lanelet::LineString3d obsStateToLs(
    const std::vector<common_msgs::TrackedObstacleState> &states) {
  lanelet::LineString3d ls{lanelet::InvalId, {}};
  for (auto state : states) {
    ls.push_back(lanelet::Point3d(lanelet::InvalId, state.pose.position.x,
                                  state.pose.position.y));
  }
  return ls;
}

LaneletSet pedestrianMovingConflicts(lanelet::LaneletLayer &layer,
                                     const common_msgs::TrackedObstacle &ped) {
  LaneletSet conflict_lls;
  for (const auto &ll : lanelet::geometry::findWithin2d(
           layer, lanelet::utils::to2D(obsStateToLs(ped.predicted_states)))) {
    conflict_lls.insert(ll.second);
  }
  return conflict_lls;
}

// Calculates how many seconds it's been since a tracked obstacle has moved 0.5
// meters from where it currently is If we've never seen it move 0.5 meters,
// returns the time that the track has been alive
ros::Duration historyStaticTime(
    std::vector<common_msgs::TrackedObstacleState> history) {
  ros::Time most_recent_stamp = history.back().header.stamp;
  ros::Time most_recent_move_stamp(0, 0);
  // We want to go from most recent to oldest, which means going through the
  // history array from back to front
  for (auto state_it = history.rbegin(); state_it != history.rend();
       ++state_it) {
    if (hypot(state_it->pose.position.x - history.back().pose.position.x,
              state_it->pose.position.y - history.back().pose.position.y) >
        0.5) {
      most_recent_move_stamp = state_it->header.stamp;
      break;
    }
  }
  if (most_recent_move_stamp.isZero()) {
    return history.back().header.stamp - history.front().header.stamp;
  }
  return most_recent_stamp - most_recent_move_stamp;
}

LaneletSet pedestrianWaitingConflicts(lanelet::LaneletLayer &layer,
                                      const common_msgs::TrackedObstacle &ped) {
  ros::Duration alive_time = ped.observation_history.back().header.stamp -
                             ped.observation_history.front().header.stamp;
  ros::Duration waiting_time = historyStaticTime(ped.observation_history);
  // If it's moved in the past 1 seconds it's not waiting
  // If it's been waiting for more than 10 seconds it's not gonna cross
  if ((alive_time.toSec() >= 1 && waiting_time.toSec() <= 1) ||
      waiting_time.toSec() > 10) {
    return {};
  }
  LaneletSet conflict_lls;
  // A waiting pedestrian conflicts with all lanelets in a 5 meter radius around
  // it
  for (const auto &ll : lanelet::geometry::findWithin2d(
           layer,
           lanelet::BasicPoint2d(ped.obstacle.pose.pose.position.x,
                                 ped.obstacle.pose.pose.position.y),
           5)) {
    conflict_lls.insert(ll.second);
  }
  return conflict_lls;
}

lanelet::Lanelets pedestrianConflicts(lanelet::LaneletLayer &layer,
                                      const common_msgs::TrackedObstacle &ped) {
  auto waiting_conflicts = pedestrianWaitingConflicts(layer, ped);
  auto moving_conflicts = pedestrianMovingConflicts(layer, ped);
  waiting_conflicts.insert(moving_conflicts.begin(), moving_conflicts.end());
  lanelet::Lanelets ret;
  ret.insert(ret.end(), waiting_conflicts.begin(), waiting_conflicts.end());
  return ret;
}

double checkStop(const lanelet::ConstLanelet &ll) {
  double stopping_dist = std::numeric_limits<double>::max();

  // Check for pedestrian regulatory elements.
  for (auto ped_reg : ll.regulatoryElementsAs<env_model::PedRegElem>()) {
    // We need to stop for this boi, but where???

    // Hybrid type needed to call into Boost geometry functions
    auto hybrid_ll_center = lanelet::traits::toHybrid(ll.centerline2d());
    auto hybrid_ped_line = lanelet::traits::toHybrid(
        lanelet::utils::to2D(ped_reg->getPredictedStates()));

    std::vector<lanelet::BasicPoint2d> intersection_points;
    // Put intersection point of ped with lanelet centerline into
    // stopping_point
    boost::geometry::intersection(hybrid_ll_center, hybrid_ped_line,
                                  intersection_points);
    ROS_WARN_COND(intersection_points.size() >= 2,
                  "Pedestrian trajectory intersects with lanelet centerline "
                  "more than once");

    lanelet::BasicPoint2d ped_stopping_point;
    // Stopping point is either the predicted intersection, or if no predicted
    // intersection, the closest point
    if (intersection_points.empty()) {
      ped_stopping_point =
          lanelet::geometry::project(ll.centerline2d(), hybrid_ped_line.front());
    } else {
      ped_stopping_point = intersection_points[0];
    }

    double ped_stopping_point_dist = lanelet::geometry::toArcCoordinates(
                                         hybrid_ll_center, ped_stopping_point)
                                         .length;
    if (ped_stopping_point_dist < stopping_dist) {
      stopping_dist = ped_stopping_point_dist - 10;
    }
  }

  // Check for stop sign regulatory elements.
  for (auto stop_sign_reg_elem :
       ll.regulatoryElementsAs<env_model::StopSignRegElem>()) {
    auto stop_sign_stopping_point = ll.centerline2d().back();
    auto stop_sign_stopping_dist = lanelet::geometry::length(ll.centerline2d());

    if (stop_sign_stopping_dist < stopping_dist) {
      if (stop_sign_reg_elem->shouldStop()) {
        stopping_dist = stop_sign_stopping_dist - 1;
      }
    }
  }
  // Check for traffic light regulatory elements.
  for (auto traffic_light_reg_elem :
       ll.regulatoryElementsAs<lanelet::TrafficLight>()) {
    auto ls = traffic_light_reg_elem->trafficLights()[0].lineString().get();
    // todo: check light status against turn direction for current and next ll
    bool has_status = ls.hasAttribute("right");
    if (!has_status) {
      continue;
    }
    bool any_green =
        (ls.attributeOr("right", "") == common_msgs::TrafficLight::TL_ST_GRE ||
         ls.attributeOr("left", "") == common_msgs::TrafficLight::TL_ST_GRE ||
         ls.attributeOr("forward", "") == common_msgs::TrafficLight::TL_ST_GRE);
    if (!any_green) {
      auto traffic_light_stopping_dist =
          lanelet::geometry::length(ll.centerline2d());
      if (traffic_light_stopping_dist < stopping_dist) {
        stopping_dist = traffic_light_stopping_dist - 1;
      }
    }
  }
  return stopping_dist;
}

std::vector<geometry_msgs::Point> lsToGeom(
    const lanelet::BasicLineString2d &ref_ls) {
  std::vector<geometry_msgs::Point> ret;
  for (const auto &p : ref_ls) {
    geometry_msgs::Point lp;
    lp.x = p.x();
    lp.y = p.y();
    ret.push_back(lp);
  }
  return ret;
}

lanelet::Point3d geomToPoint(const geometry_msgs::Point p, bool allocate_id) {
  return lanelet::Point3d{
      allocate_id ? lanelet::utils::getId() : lanelet::InvalId, p.x, p.y, p.z};
}

geometry_msgs::Point pointToGeom(const lanelet::ConstPoint3d &p) {
  geometry_msgs::Point ret;
  ret.x = p.x();
  ret.y = p.y();
  ret.z = p.z();
  return ret;
}

geometry_msgs::Point pointToGeom(const lanelet::BasicPoint3d &p) {
  geometry_msgs::Point ret;
  ret.x = p.x();
  ret.y = p.y();
  ret.z = p.z();
  return ret;
}

geometry_msgs::Point pointToGeom(const lanelet::ConstPoint2d &p) {
  geometry_msgs::Point ret;
  ret.x = p.x();
  ret.y = p.y();
  ret.z = 0;
  return ret;
}

geometry_msgs::Point pointToGeom(const lanelet::BasicPoint2d &p) {
  geometry_msgs::Point ret;
  ret.x = p.x();
  ret.y = p.y();
  ret.z = 0;
  return ret;
}

visualization_msgs::MarkerArray createRefLineMarker(
    const std::vector<geometry_msgs::Point> &ref) {
  visualization_msgs::MarkerArray ret;
  if (ref.empty()) {
    return ret;
  }
  visualization_msgs::Marker mark;
  mark.header.frame_id = "odom";
  mark.points = ref;
  mark.ns = "local_ref";
  mark.type = mark.LINE_STRIP;
  mark.scale.x = 0.5;
  mark.pose.position.z += 0.1;
  mark.color.a = 1.0;
  mark.color.r = 1.0;
  mark.color.g = 1.0;
  mark.color.b = 102 / 255;
  mark.lifetime = ros::Duration(1);
  ret.markers.push_back(mark);
  return ret;
}

visualization_msgs::MarkerArray regElemMarkers(const lanelet::Lanelet &ll) {
  visualization_msgs::MarkerArray ret;
  auto peds = ll.regulatoryElementsAs<path_planning::env_model::PedRegElem>();
  if (!peds.empty()) {
    std_msgs::ColorRGBA c;
    c.a = 1;
    c.r = 1;
    auto ll_markers =
        lanelet::visualization::laneletsBoundaryAsMarkerArray({ll}, c, false);
    for (auto marker : ll_markers.markers) {
      marker.pose.position.z += 0.1;
      marker.id *= -1;
      marker.lifetime = ros::Duration(1);
      marker.header.frame_id = "odom";
      ret.markers.push_back(marker);
    }
    c.g = 1;
    for (auto ped : peds) {
      auto ped_marker = lanelet::visualization::lineStringsAsMarkerArray(
                            {ped->getPredictedStates()}, "ped_reg_elem", c, 1)
                            .markers[0];
      ped_marker.lifetime = ros::Duration(1);
      ped_marker.id = ped->id();
      ped_marker.header.frame_id = "odom";
      ret.markers.push_back(ped_marker);
    }
  }
  auto trafficSigns = ll.regulatoryElementsAs<lanelet::TrafficSign>();
  if (!trafficSigns.empty()) {
    std_msgs::ColorRGBA c;
    c.a = 1;
    c.b = 1;

    auto ll_markers = lanelet::visualization::laneletsBoundaryAsMarkerArray(
        {ll}, c, false);  // lanelet into rviz
    for (auto marker : ll_markers.markers) {
      marker.scale.x = 0.4;
      marker.pose.position.z += 0.05;
      marker.header.frame_id = "odom";
      marker.id *= -2;
      marker.ns = "traffic_sign_reg_elem";
      marker.lifetime = ros::Duration(1);
      ret.markers.push_back(marker);
    }
  }
  auto trafficLights = ll.regulatoryElementsAs<lanelet::TrafficLight>();
  if (!trafficLights.empty()) {
    std_msgs::ColorRGBA c;
    c.a = 1;
    c.r = 1;
    c.g = 191.0 / 255.0;
    c.b = 128.0 / 255.0;

    auto ll_markers = lanelet::visualization::laneletsBoundaryAsMarkerArray(
        {ll}, c, false);  // lanelet into rviz
    for (auto marker : ll_markers.markers) {
      marker.scale.x = 0.4;
      marker.pose.position.z += 0.05;
      marker.id *= -2;
      marker.header.frame_id = "odom";
      marker.ns = "traffic_light_reg_lanelet";
      marker.lifetime = ros::Duration(1);
      ret.markers.push_back(marker);
    }
    for (auto &tl : trafficLights[0]->trafficLights()) {
      auto tl_m = lanelet::visualization::lineStringsAsMarkerArray(
                      {tl.lineString().get()}, "traffic_light_reg_elem", c, 0.1)
                      .markers[0];
      tl_m.header.frame_id = "odom";
      tl_m.ns = "traffic_light_reg_elem";
      tl_m.lifetime = ros::Duration(1);
      ret.markers.push_back(tl_m);
    }
  }
  return ret;
}

visualization_msgs::MarkerArray mapMarkers(lanelet::LaneletMapPtr map) {
  visualization_msgs::MarkerArray ret;
  lanelet::ConstLanelets lanelets(map->laneletLayer.begin(),
                                  map->laneletLayer.end());
  std_msgs::ColorRGBA c;
  c.a = 1;
  c.g = 1;
  auto ll_marker_array =
      lanelet::visualization::laneletsBoundaryAsMarkerArray(lanelets, c, true);
  for (auto marker : ll_marker_array.markers) {
    marker.header.frame_id = "odom";
    marker.ns = "map";
    ret.markers.push_back(marker);
  }
  return ret;
}

visualization_msgs::MarkerArray createRouteMarker(
    const lanelet::routing::LaneletPath &route) {
  visualization_msgs::MarkerArray ret;
  std_msgs::ColorRGBA c;
  c.a = 0.75;
  c.r = 1;
  c.g = 179.0 / 255.0;
  c.b = 1;
  for (const auto &ll : route) {
    auto m = lanelet::visualization::lineStringsAsMarkerArray(
                 {ll.centerline3d()}, "global_route", c, 0.5)
                 .markers[0];
    m.header.frame_id = "odom";
    m.id = -3 * ll.id();
    m.scale.x = 0.5;
    ret.markers.push_back(m);
  }
  return ret;
}

visualization_msgs::MarkerArray createDestinationMarker(
    const DestinationList &dl) {
  visualization_msgs::MarkerArray ret;
  std_msgs::ColorRGBA c;
  c.a = 1;
  c.r = 204.0 / 255.0;
  c.g = 204.0 / 255.0;
  c.b = 0;
  int id = 0;
  for (const auto &d : dl) {
    visualization_msgs::Marker m;
    m.color = c;
    m.header.frame_id = "odom";
    m.ns = "destinations";
    m.id = id;
    id++;
    m.type = m.CYLINDER;
    m.scale.x = 1;
    m.scale.y = 1;
    m.scale.z = 2;
    m.pose.position.x = d.second.x();
    m.pose.position.y = d.second.y();
    ret.markers.push_back(m);
  }
  return ret;
}

std::vector<geometry_msgs::Point> postProessRef(
    std::vector<geometry_msgs::Point> &ref, double back_dist) {
  if (ref.size() < 2) {
    return ref;
  }
  double dy = ref[1].y - ref[0].y;
  double dx = ref[1].x - ref[0].x;
  double mag = sqrt(dy * dy + dx * dx);

  geometry_msgs::Point np;
  np.y = ref[0].y - back_dist * (dy / mag);
  np.x = ref[0].x - back_dist * (dx / mag);
  ref.insert(ref.begin(), np);
  std::vector<geometry_msgs::Point> ret;
  for (int i = 0; i < ref.size() - 1; i++) {
    dy = ref[i + 1].y - ref[i].y;
    dx = ref[i + 1].x - ref[i].x;
    mag = sqrt(dy * dy + dx * dx);
    int num_interp = mag / 0.5 + 1;
    for (double j = 0; j < num_interp; j += 1) {
      np.y = ref[i].y + (j / num_interp) * dy;
      np.x = ref[i].x + (j / num_interp) * dx;
      ret.push_back(np);
    }
  }
  ret.push_back(ref.back());
  return ret;
}

visualization_msgs::MarkerArray deleteAllMarker() {
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  visualization_msgs::MarkerArray ma;
  ma.markers.push_back(mark);
  return ma;
}

}  // namespace utils
}  // namespace env_model
}  // namespace path_planning