#include "pp_env_model/env_model.hpp"

using namespace path_planning::env_model;

EnvModel::EnvModel(lanelet::LaneletMapPtr lanelet_map, ros::NodeHandle &nh)
    : lanelet_map_{lanelet_map},
      router{lanelet_map},
      route_viz_pub{nh.advertise<visualization_msgs::MarkerArray>(
          "/path_planning/route_viz", 1)},
      filtered_occ_pub_{nh.advertise<nav_msgs::OccupancyGrid>(
          "/path_planning/filtered_occ", 1)},
      tf_listener_ptr_{
          std::make_unique<tf2_ros::TransformListener>(tf_buffer_)} {
  ROS_INFO_STREAM("Env Model initialized with "
                  << lanelet_map_->laneletLayer.size() << " lanelets.");
}

lanelet::LaneletMapPtr EnvModel::map() { return lanelet_map_; }

DistLaneletList EnvModel::getEgoLanelets() {
  geometry_msgs::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(kMapFrameName, kVechielFrameName,
                                    ros::Time(0), ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    return {};
  }
  auto ego_lanelets = lanelet::geometry::findWithin2d(
      lanelet_map_->laneletLayer,
      lanelet::Point2d{lanelet::InvalId, tf.transform.translation.x,
                       tf.transform.translation.y});

  return ego_lanelets;
}

bool EnvModel::route() {
  bool route_res = router.route(getEgoLanelets());
  if (route_res) {
    route_viz_pub.publish(env_model::utils::deleteAllMarker());
    route_viz_pub.publish(
        env_model::utils::createRouteMarker(router.getRoute()));
    route_viz_pub.publish(
        env_model::utils::createDestinationMarker(router.getDesinations()));
  }

  return route_res;
}

// Called from traffic_signs_callback
void EnvModel::stopSignAssociation(
    const common_msgs::TrafficSign &traffic_sign) {
  // According to the Wikipedia page for lanes, the max lane width is 4.6 m,
  // but are generally narrower for low capacity roads. A search radius of 9.2
  // m is employed to find the nearest lanelet in the controlled lanelet set
  // based on the width of one lanelet and the width of an additional lanelet
  // to account for sign placement. https://en.wikipedia.org/wiki/Lane
  auto candidate_lanelets = lanelet::geometry::findWithin2d(
      lanelet_map_->laneletLayer,
      lanelet::Point2d{lanelet::InvalId, traffic_sign.pose.position.x,
                       traffic_sign.pose.position.y},
      kStopSignSearchRad);

  LaneletSet candidate_lanelets_set;
  for (auto candidate_ll : candidate_lanelets) {
    auto besideLanelets = router.graph()->besides(candidate_ll.second);

    for (auto ll : besideLanelets) {
      candidate_lanelets_set.insert(lanelet_map_->laneletLayer.get(ll.id()));
    }
  }

  // Lanelets of the ego vehicle.
  auto ego_lanelets = getEgoLanelets();
  if (ego_lanelets.empty()) {
    ROS_WARN(
        "While associating stop sign, failed to get ego lanelets, aborting.");
    return;
  }

  LaneletSet controlled_lanelets;
  for (auto ego_ll : ego_lanelets) {
    for (auto candidate_ll : candidate_lanelets_set) {
      if (ego_ll.second == candidate_ll) {
        controlled_lanelets.insert(ego_ll.second);
      }
    }
  }

  if (controlled_lanelets.empty()) {
    return;
  }

  lanelet::ConstLanelets besideLanelets;
  LaneletSet controlled_lanelets_and_besides_set;
  for (auto controlled_ll : controlled_lanelets) {
    besideLanelets = router.graph()->besides(controlled_ll);
    for (auto ll : besideLanelets) {
      controlled_lanelets_and_besides_set.insert(
          lanelet_map_->laneletLayer.get(ll.id()));
    }
  }
  lanelet::ConstPoint3d cleft_point = besideLanelets.front().leftBound().back();
  lanelet::ConstPoint3d cright_point =
      besideLanelets.back().rightBound().back();
  lanelet::Point3d left_point{lanelet::utils::getId(), cleft_point.x(),
                              cleft_point.y(), cleft_point.z()};
  lanelet::Point3d right_point{lanelet::utils::getId(), cright_point.x(),
                               cright_point.y(), cright_point.z()};
  lanelet::LineString3d ls(lanelet::utils::getId(), {left_point, right_point});
  lanelet::Id stop_sign_id = lanelet::utils::getId();
  tracker_map_id_[traffic_sign.id] = stop_sign_id;
  lanelet::Point3d stop_sign_loc(lanelet::utils::getId(),
                                 traffic_sign.pose.position.x,
                                 traffic_sign.pose.position.y, 0);
  lanelet::RuleParameterMap params{{lanelet::RoleNameString::RefLine, {ls}}};
  lanelet::RegulatoryElementPtr stop_sign =
      lanelet::RegulatoryElementFactory::create("stop_sign", stop_sign_id,
                                                params);

  for (auto ll : controlled_lanelets_and_besides_set) {
    ll.addRegulatoryElement(stop_sign);
  }
  lanelet_map_->add(stop_sign);
}

void EnvModel::trafficLightDetectionCallback(
    const common_msgs::TrafficLight &tl_status) {
  if (lanelet_map_->regulatoryElementLayer.find(tl_status.id) ==
      lanelet_map_->regulatoryElementLayer.end()) {
    ROS_WARN_STREAM_THROTTLE(1,
                             "In trafficLightDetectionCallback, tl_status.id ["
                                 << tl_status.id << "] not in map");
    return;
  }
  lanelet::RegulatoryElementPtr re_ptr =
      lanelet_map_->regulatoryElementLayer.get(tl_status.id);
  auto ls = std::dynamic_pointer_cast<lanelet::TrafficLight>(re_ptr)
                ->trafficLights()[0]
                .lineString()
                .get();
  ls.setAttribute("right", tl_status.right);
  ls.setAttribute("left", tl_status.left);
  ls.setAttribute("forward", tl_status.forward);
}

void EnvModel::trafficSignCallback(
    const common_msgs::TrafficSignList &traffic_sign_list) {
  auto signsList = traffic_sign_list.traffic_signs;
  std::vector<common_msgs::TrafficSign> newSignsOnlyList;

  for (auto signObj : signsList) {
    // unique ID (given from perception) already exists - not a new sign
    if (tracker_map_id_.find(signObj.id) != tracker_map_id_.end()) {
      continue;
    }
    if (signObj.traffic_sign_type == common_msgs::TrafficSign::TS_TP_STOP) {
      stopSignAssociation(signObj);
      continue;
    }
    newSignsOnlyList.push_back(signObj);
  }

  if (newSignsOnlyList.empty()) {
    return;
  }

  intersectionSignsAssociation(newSignsOnlyList);
  ROS_DEBUG_STREAM("End of callback");
}

double EnvModel::distanceFromStartToClosestPoint(
    lanelet::BasicPoint2d start,
    std::vector<lanelet::BasicPoint2d> obstacle_points) {
  double distance = INT_MAX;
  for (auto point : obstacle_points) {
    lanelet::BasicLineString2d l;
    l.push_back(start);
    l.push_back(point);
    double distanceToPoint = boost::geometry::length(l);
    if (distanceToPoint < distance) {
      distance = distanceToPoint;
    }
  }
  return distance;
}

//
void EnvModel::lateral_shift(const nav_msgs::OccupancyGrid &occupancy_grid) {
  geometry_msgs::TransformStamped base_link_map_tf;
  try {
    base_link_map_tf = tf_buffer_.lookupTransform(
        kMapFrameName, kVechielFrameName, ros::Time(0), ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN_STREAM("Base link map transform exception in lateral shift");
    return;
  }
  lanelet::Point3d ego_location = lanelet::Point3d(
      lanelet::InvalId, base_link_map_tf.transform.translation.x,
      base_link_map_tf.transform.translation.y,
      base_link_map_tf.transform.translation.z);
  // get closest lanelet on curr_path to that location
  int curr_lanelet_index = router.closestLaneletOnRoute(ego_location, true);
  if (curr_lanelet_index == -1 || curr_lanelet_index + 1 >= router.getRoute().size()) {
    return;
  }
  lanelet::Optional<int> point_i;
  lanelet::ConstLanelet next_ll_on_route =
      router.getRoute()[curr_lanelet_index + 1];
  for (int i = 0; i < occupancy_grid.data.size(); i++) {
    if (occupancy_grid.data[i] == 0) {
      continue;
    }
    auto odomp = utils::occupancyToPoint(i, occupancy_grid);
    if (!odomp) {
      continue;
    }
    if (lanelet::geometry::inside(
            next_ll_on_route, utils::geomToPoint(odomp.get()).basicPoint2d())) {
      point_i = i;
      break;
    }
  }
  if (!point_i) {
    return;
  }

  lanelet::ConstLineString2d centerline = next_ll_on_route.centerline2d();
  double safety_tolerance = 0.2;
  double left_shift = 0.0, right_shift = 0.0;

  // bfs to traverse the grid from the initial point
  std::unordered_set<int> visited;
  std::queue<int> q;
  visited.insert(point_i.get());
  q.push(point_i.get());

  // all points in q are occupied and non-visited
  while (!q.empty()) {
    int index = q.front();
    q.pop();

    int num_grid_x = occupancy_grid.info.width;   // x is longitudinal direction
    int num_grid_y = occupancy_grid.info.height;  // y is lateral direction
    lanelet::BasicPoint2d _point =
        lanelet::utils::to2D(
            utils::geomToPoint(
                utils::occupancyToPoint(index, occupancy_grid).value()))
            .basicPoint2d();

    double signedDistanceToCenterline =
        lanelet::geometry::signedDistance(centerline, _point);
    ROS_DEBUG_STREAM("point: " << _point.x() << " : " << _point.y()
                               << "with distance: "
                               << signedDistanceToCenterline);
    if (signedDistanceToCenterline > 0) {
      signedDistanceToCenterline +=
          safety_tolerance + occupancy_grid.info.resolution / 2;
      left_shift = left_shift < signedDistanceToCenterline
                       ? signedDistanceToCenterline
                       : left_shift;
    } else {
      signedDistanceToCenterline -=
          (safety_tolerance + occupancy_grid.info.resolution / 2);
      right_shift = right_shift < -signedDistanceToCenterline
                        ? -signedDistanceToCenterline
                        : right_shift;
    }

    std::vector<int> adj;
    if (index % num_grid_y == 0) {
      std::vector<int> temp{index + 1, index + num_grid_y, index - num_grid_y,
                            index + num_grid_y + 1, index - num_grid_y + 1};
      adj = temp;
    } else if ((index + 1) % num_grid_y == 0) {
      std::vector<int> temp{index - 1, index + num_grid_y, index - num_grid_y,
                            index + num_grid_y - 1, index - num_grid_y - 1};
      adj = temp;
    } else {
      std::vector<int> temp{index + 1,
                            index - 1,
                            index + num_grid_y,
                            index - num_grid_y,
                            index + num_grid_y + 1,
                            index + num_grid_y - 1,
                            index - num_grid_y - 1,
                            index - num_grid_y + 1};
      adj = temp;
    }

    for (auto p : adj) {
      if (0 > p || p >= num_grid_y * num_grid_x ||
          (int)occupancy_grid.data[p] <= 50 ||
          visited.find(p) != visited.end()) {
        continue;
      }
      q.push(p);
      visited.insert(p);
    }
  }
  double width =
      lanelet::geometry::distance2d(next_ll_on_route.leftBound2d(),
                                    next_ll_on_route.rightBound2d()) /
      2;
  ROS_DEBUG_STREAM("w = " << width << " and l = " << left_shift
                          << " and r = " << right_shift);
  if (width < left_shift) {
    left_shift = INT_MAX;
  }
  if (width < abs(right_shift)) {
    right_shift = INT_MAX;
  }
  // if both left and right are infinity, it is blocked
  if (left_shift == INT_MAX && abs(right_shift) == INT_MAX) {
    auto non_const_ll = lanelet_map_->laneletLayer.get(next_ll_on_route.id());
    if (non_const_ll.hasAttribute("blocked_by_occupancy")) {
      return;
    }
    non_const_ll.setAttribute("blocked_by_occupancy", true);
    router.build(lanelet_map_);
    ROS_INFO_STREAM("Routing graph rebuilt with"
                    << lanelet_map_->laneletLayer.size() << " lanelets.");
    for (auto err : router.graph()->checkValidity()) {
      ROS_WARN_STREAM("Routing Graph Error: " << err);
    }
    if (!route()) {
      ROS_WARN_THROTTLE(1, "After blocked lanelet handling, routing failed");
    }
  }
}

void EnvModel::occGridCallback(const nav_msgs::OccupancyGrid &occupancy_grid) {
  nav_msgs::OccupancyGrid occ_filt = occupancy_grid;
  for (int i = 0; i < occ_filt.data.size(); i++) {
    if (occ_filt.data[i] == 0) {
      continue;
    }
    auto odomp = utils::occupancyToPoint(i, occ_filt);
    if (!odomp) {
      continue;
    }
    bool in_route = false;
    for (const auto &ll : router.getRoute()) {
      auto besides = router.graph()->besides(ll);
      for (const auto &bll : besides) {
        if (lanelet::geometry::inside(
                bll, utils::geomToPoint(odomp.get()).basicPoint2d())) {
          in_route = true;
        }
      }
    }
    if (!in_route) {
      occ_filt.data[i] = 0;
    }
  }
  filtered_occ_pub_.publish(occ_filt);
  lateral_shift(occupancy_grid);
}

void EnvModel::intersectionSignsAssociation(
    std::vector<common_msgs::TrafficSign> signs) {
  // Get current ego lanelet
  auto curr_lls = getEgoLanelets();
  if (curr_lls.empty()) {
    ROS_WARN(
        "While associating intersection traffic signs to lanelet, failed to "
        "localize ego vehicle in lanelets, aborting");
    return;
  }

  // Collect adjacent (lateral) lanelets - includes the current lanelet(s),
  // doesn't collect the opposite direction lanelets
  lanelet::ConstLanelets allBeside;
  for (auto cur_ll : curr_lls) {
    auto temp = router.graph()->besides(cur_ll.second);
    allBeside.insert(allBeside.end(), temp.begin(), temp.end());
  }

  ROS_DEBUG_STREAM("One of the current lanelets && Adjacent Lanelet List Size "
                   << curr_lls[0].second.id() << " " << allBeside.size());
  for (auto ll : allBeside) {
    ROS_DEBUG_STREAM("Adjacent List: " << ll.id());
  }

  ConstLaneletSet turnDirCandidateLLs;

  // Perform BFS with max value of "search_Depth" with the goal of finding a
  // layer containing a lanelet with a "turn_direction" attribute
  for (auto adjacent : allBeside) {
    router.seachForTurnDir(
        adjacent, turnDirCandidateLLs);  // pass by reference updates the set
  }

  ROS_DEBUG_STREAM("Number of intersection lanelets (With turn_dir) "
                   << turnDirCandidateLLs.size());

  // Loop through the new signs and perform RegElem association using
  // predetermined turning lane tags (HDmaps)

  lanelet::RegulatoryElementPtr RegElemPtr = nullptr;
  for (auto signObj : signs) {
    for (auto immutableLanelet : turnDirCandidateLLs) {
      auto id = immutableLanelet.id();
      auto mutableLanelet = *(lanelet_map_->laneletLayer.find(id));

      if (utils::signIsRelevantToLanelet(mutableLanelet, signObj)) {
        RegElemPtr = utils::buildTrafficSignRegElem(signObj);
        mutableLanelet.addRegulatoryElement(
            RegElemPtr);  // RegElem can be later distinguised by the
                          // signTypeObject which contains location and signType
        tracker_map_id_[signObj.id] =
            RegElemPtr->id();  // store traffic sign in the master map, won't
                               // want to repeat associations
      }
    }
  }

  if (RegElemPtr) {
    router.build(lanelet_map_);
    ROS_INFO_STREAM("Routing graph rebuilt with"
                    << lanelet_map_->laneletLayer.size() << " lanelets.");
    for (auto err : router.graph()->checkValidity()) {
      ROS_WARN_STREAM("Routing Graph Error: " << err);
    }
    if (!route()) {
      ROS_WARN_THROTTLE(1, "After intersection sign handling, routing failed");
    }
  }
}

void EnvModel::trackedObstacleCallback(
    const common_msgs::TrackedObstacleList &tracked_obs_list) {
  for (auto tracked_obs : tracked_obs_list.tracked_obstacles) {
    // ignore if obs not a pedestrian
    if (tracked_obs.obstacle.label != common_msgs::Obstacle::OBS_TP_PED) {
      continue;
    }
    auto conflict_lls =
        utils::pedestrianConflicts(lanelet_map_->laneletLayer, tracked_obs);
    auto predicted_ped_states =
        utils::obsStateToLs(tracked_obs.predicted_states);
    // If we already have this ped in the map, update its conflicts
    if (tracker_map_id_.find(tracked_obs.obstacle.object_id) !=
        tracker_map_id_.end()) {
      // Get ped already in map by its ID
      lanelet::RegulatoryElementPtr ped =
          lanelet_map_->regulatoryElementLayer.get(
              tracker_map_id_[tracked_obs.obstacle.object_id]);
      std::dynamic_pointer_cast<env_model::PedRegElem>(ped)
          ->updatePredictedStates(predicted_ped_states);
      // Find lanelets that own the ped
      for (auto &ll : lanelet_map_->laneletLayer) {
        auto peds = ll.regulatoryElementsAs<env_model::PedRegElem>();
        bool owns_ped = std::find(peds.begin(), peds.end(), ped) != peds.end();
        bool should_own_ped =
            std::find(conflict_lls.begin(), conflict_lls.end(), ll) !=
            conflict_lls.end();
        if (owns_ped && !should_own_ped) {
          ll.removeRegulatoryElement(ped);
        } else if (!owns_ped && should_own_ped) {
          ll.addRegulatoryElement(ped);
        }
      }
    }
    // Else create reg elem for ped and add to map
    else {
      lanelet::Id reg_elem_id = lanelet::utils::getId();
      tracker_map_id_[tracked_obs.obstacle.object_id] = reg_elem_id;
      lanelet::RuleParameterMap params{
          {lanelet::RoleNameString::RefLine, {predicted_ped_states}}};
      lanelet::RegulatoryElementPtr ped =
          lanelet::RegulatoryElementFactory::create("pedestrian", reg_elem_id,
                                                    params);
      lanelet_map_->add(ped);
      for (auto &ll : conflict_lls) {
        ll.addRegulatoryElement(ped);
      }
    }
  }
}

void EnvModel::pruneDeadTracks(const ros::TimerEvent &) {
  for (auto &ll : lanelet_map_->laneletLayer) {
    auto peds = ll.regulatoryElementsAs<env_model::PedRegElem>();
    if (peds.empty()) {
      continue;
    }
    for (auto &ped : peds) {
      if (ped->isTrackDead()) {
        ll.removeRegulatoryElement(ped);
      }
    }
  }
}

lanelet::Optional<common_msgs::TrafficLightList> EnvModel::nextTrafficLight() {
  // get ego location as point3d
  geometry_msgs::TransformStamped base_link_map_tf;
  try {
    base_link_map_tf = tf_buffer_.lookupTransform(
        kMapFrameName, kVechielFrameName, ros::Time(0), ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN_STREAM("Base link map transform exception in traffic light timer");
    return {};
  }
  lanelet::Point3d ego_location = lanelet::Point3d(
      lanelet::InvalId, base_link_map_tf.transform.translation.x,
      base_link_map_tf.transform.translation.y,
      base_link_map_tf.transform.translation.z);
  // get closest lanelet on curr_path to that location
  int curr_lanelet_index = router.closestLaneletOnRoute(ego_location);
  if (curr_lanelet_index == -1) {
    return {};
  }
  lanelet::LaneletSequence remaining_path = router.routeToFirstTrafficLight(
      curr_lanelet_index);  // should always return something unless all LLs in
                            // curr_path over maxdbl

  // Get list of all traffic lights on remaining path
  auto tl_on_remaining_path =
      remaining_path.regulatoryElementsAs<lanelet::TrafficLight>();
  if (tl_on_remaining_path.size() > 0) {
    auto tl_boxes = tl_on_remaining_path[0]->trafficLights();
    common_msgs::TrafficLightList tl_list_msg;
    tl_list_msg.id = tl_on_remaining_path[0]->id();
    for (auto tl_linestring : tl_boxes) {
      if (tl_linestring.isLineString()) {
        // Linestrings are 3 points that define a plane
        // [1] and [2] are diagonal
        auto p1 = tl_linestring.lineString().get()[1];
        common_msgs::TrafficLight tl_msg;
        tl_msg.header.frame_id = "odom";
        tl_msg.p1.x = p1.x();
        tl_msg.p1.y = p1.y();
        tl_msg.p1.z = p1.z();

        auto p2 = tl_linestring.lineString().get()[2];
        tl_msg.p2.x = p2.x();
        tl_msg.p2.y = p2.y();
        tl_msg.p2.z = p2.z();
        tl_msg.id = tl_on_remaining_path[0]->id();
        tl_list_msg.traffic_lights.push_back(tl_msg);
      } else {
        ROS_WARN("Lanelet traffic light tl_linestring is not a linestring!");
      }
    }
    return tl_list_msg;
  }
  return {};
}


path_planning_msgs::ReferenceLine EnvModel::localRef(
    const lanelet::Point3d &ego_location) {
  lanelet::routing::LaneletPath route = router.getRoute();
  std::pair<lanelet::ConstLanelet, double> route_end = router.routeEnd();
  path_planning_msgs::ReferenceLine ref;

  int start_ll_i = router.closestLaneletOnRoute(ego_location, true);
  
  if (router.getRoute().empty() || start_ll_i < 0) {
    return ref;
    //Returns Empty ReferenceLine Message 
  }

  auto ego_p = lanelet::geometry::project(route[start_ll_i].centerline2d(), lanelet::utils::to2D(ego_location));
  auto besides = router.graph()->besides(route[start_ll_i]);
  lanelet::Lanelets other_side_lls = lanelet_map_->laneletLayer.findUsages(besides[0].leftBound().invert());
  lanelet::BasicLineString2d left_bound;

  if (!other_side_lls.empty()) { //If There Is Opposite Lanelet , Find Right Bound of Right_Most Opposite Lanelet
    left_bound = router.graph()->besides(other_side_lls[0]).back().rightBound2d().basicLineString();
  }else{ //If There is NO Opposite Lanelet, Find Left Bound of Left-Most Lanelet
    left_bound = besides[0].leftBound2d().basicLineString();
  }
  lanelet::BasicLineString2d right_bound = besides.back().rightBound2d().basicLineString();
  ref.max_left_dev = lanelet::geometry::distance2d(ego_p, left_bound);
  ref.max_right_dev = lanelet::geometry::distance2d(ego_p, right_bound);

  ROS_DEBUG_STREAM("***************************");
  ROS_DEBUG_STREAM("start_ll_i: " << start_ll_i);
  double lookahead_remain = kLocalLookAhead;
  for (unsigned int ll_i = start_ll_i; ll_i < route.size(); ll_i++) {
    ROS_DEBUG_STREAM("ll_i: " << ll_i);
    lanelet::ConstLanelet curr_ll = route[ll_i];
    if (ll_i < route.size() - 1 &&
        router.laneChange(curr_ll, route[ll_i + 1])) {
      continue;
    }
    lanelet::ConstLineString2d curr_center = curr_ll.centerline2d();
    double start_dist = 0;
    if (start_ll_i == ll_i)
      start_dist = lanelet::geometry::toArcCoordinates(
                       curr_center, ego_location.basicPoint2d())
                       .length;
    ROS_DEBUG_STREAM("start_dist: " << start_dist);
    double reg_elem_stop = utils::checkStop(curr_ll);
    ROS_DEBUG_STREAM("reg_elem_stop: " << reg_elem_stop);
    double stop_dist = std::min(lookahead_remain + start_dist, reg_elem_stop);
    ROS_DEBUG_STREAM("lookahead_remain + start_dist: " << lookahead_remain +
                                                              start_dist);
    if (curr_ll == route_end.first) {
      stop_dist = std::min(stop_dist, route_end.second);
      ROS_DEBUG_STREAM("On last ll, route_end.second: " << route_end.second);
    } else {
      ROS_DEBUG_STREAM("Not on last ll");
    }
    ROS_DEBUG_STREAM("stop_dist: " << stop_dist);
    if (start_dist >= stop_dist - 0.1) {
      if (ll_i == start_ll_i) {
        ROS_WARN_STREAM_THROTTLE(
            1, "Ego is " << start_dist - stop_dist << "m past desired stop.");
      }
      ref.ref_line = utils::postProessRef(ref.ref_line, kLocalLookBack);
      return ref;
      // return local_ref ReferenceLine message
      // @return ReferenceLine {header=NULL, ref_line, max_left_dev, max_right_dev}
    }

    double curr_dist = 0;
    for (unsigned int p_i = 0; p_i < curr_center.size(); p_i++) {
      if (p_i == curr_center.size() - 1) {
        ref.ref_line.push_back(utils::pointToGeom(curr_center[p_i]));
        continue;
      }
      double next_dist =
          curr_dist +
          lanelet::geometry::distance2d(curr_center[p_i], curr_center[p_i + 1]);

      ROS_DEBUG_STREAM("p_i: " << p_i << ", next_dist: " << next_dist);

      if (next_dist >= stop_dist) {
        if (curr_dist < start_dist) {
          ref.ref_line.push_back(
              utils::pointToGeom(lanelet::geometry::interpolatedPointAtDistance(
                  curr_center, start_dist)));
        } else {
          ref.ref_line.push_back(utils::pointToGeom(curr_center[p_i]));
        }
        ref.ref_line.push_back(
            utils::pointToGeom(lanelet::geometry::interpolatedPointAtDistance(
                curr_center, stop_dist)));
        ref.ref_line = utils::postProessRef(ref.ref_line, kLocalLookBack);
        return ref;
        // return local_ref ReferenceLine message
        // @return ReferenceLine {header=NULL, ref_line, max_left_dev, max_right_dev}
      } else if (curr_dist < start_dist && next_dist > start_dist) {
        ref.ref_line.push_back(
            utils::pointToGeom(lanelet::geometry::interpolatedPointAtDistance(
                curr_center, start_dist)));
      } else if (curr_dist >= start_dist && curr_dist <= stop_dist) {
        ref.ref_line.push_back(utils::pointToGeom(curr_center[p_i]));
      }
      curr_dist = next_dist;
    }
    lookahead_remain -= (curr_dist - start_dist);
    ROS_DEBUG_STREAM("(curr_dist - start_dist): " << (curr_dist - start_dist));
  }

  ref.ref_line = utils::postProessRef(ref.ref_line, kLocalLookBack);
  return ref;
  // return local_ref ReferenceLine message
  // @return ReferenceLine {header=NULL, ref_line, max_left_dev, max_right_dev}
}

void EnvModel::updateStopSigns(nav_msgs::Odometry odom) {
  int route_pos = router.closestLaneletOnRoute(
      utils::geomToPoint(odom.pose.pose.position), true);
  if (route_pos < 0) {
    return;
  }
  for (auto &constss :
       router.getRoute()[route_pos].regulatoryElementsAs<StopSignRegElem>()) {
    ROS_DEBUG("Stop sign on route");
    lanelet::RegulatoryElementPtr ss =
        lanelet_map_->regulatoryElementLayer.get(constss->id());
    std::dynamic_pointer_cast<env_model::StopSignRegElem>(ss)->pushEgoOdom(
        odom);
  }
  return;
}
