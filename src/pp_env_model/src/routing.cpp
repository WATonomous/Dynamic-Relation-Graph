#include "pp_env_model/routing.hpp"

using namespace path_planning::env_model;

template <typename T>
std::ostream &operator<<(std::ostream &output, std::vector<T> const &values) {
  for (auto const &value : values) {
    output << value << ", ";
  }
  return output;
}

Router::Router(lanelet::LaneletMapPtr map)
    : traffic_rules_{lanelet::traffic_rules::TrafficRulesFactory::create(
          "wato", lanelet::Participants::Vehicle)} {
  build(map);
}

void Router::build(lanelet::LaneletMapPtr map) {
  routing_graph_ = lanelet::routing::RoutingGraph::build(*map, *traffic_rules_);
  for (auto err : routing_graph_->checkValidity()) {
    ROS_WARN_STREAM("Routing Graph Error: " << err);
  }
}

lanelet::routing::RoutingGraphPtr Router::graph() { return routing_graph_; }

void Router::setDesinations(DestinationList dests) { dest_list_ = dests; }

Destination Router::currDest() {
  Destination ret;
  if (!dest_list_.empty()) {
    ret = dest_list_[0];
  }
  return ret;
}

// Helper function for traffic_signs_callback: BFS with max value of
// "search_Depth" with the goal of finding a layer containing a lanelet with a
// "turn_direction" attribute
void Router::seachForTurnDir(lanelet::ConstLanelet &adjacent,
                             ConstLaneletSet &turnDirCandidateLLs) {
  if (adjacent.hasAttribute(
          "turn_direction")) {  // Will need to mod this if HDMaps gives every
                                // lanelet a turn_direction
    turnDirCandidateLLs.insert(adjacent);
    ROS_DEBUG_STREAM("Curr Lanelet has turn_direction attribute "
                     << adjacent.id());
    return;
  }

  bool turn_dir_detected = false;
  auto next_layer_successors = routing_graph_->following(adjacent);
  ROS_DEBUG_STREAM("Successors List Size: " << next_layer_successors.size());

  int search_Depth = 0;
  while (turn_dir_detected == false &&
         search_Depth != kMaxSearchDepthForIntersectionLlBFS) {
    auto successor_Lls = next_layer_successors;
    next_layer_successors.clear();

    for (auto successor : successor_Lls) {
      ROS_DEBUG_STREAM("Looping through successor list: " << successor.id());

      if (successor.hasAttribute("turn_direction")) {
        turnDirCandidateLLs.insert(successor);
        turn_dir_detected =
            true;  // found the turn_direction for this list of successors,
                   // finish off this lateral layer and stop expanding search
        ROS_DEBUG_STREAM("Successor Lanelet has turn_direction attribute "
                         << successor.id());
      }

      if (turn_dir_detected == false) {
        auto nextSuccessors = routing_graph_->following(successor);
        ROS_DEBUG_STREAM("First, next successor "
                         << nextSuccessors[0].id()
                         << " Next Successors List Size "
                         << nextSuccessors.size());
        next_layer_successors.insert(next_layer_successors.end(),
                                     nextSuccessors.begin(),
                                     nextSuccessors.end());
      }
    }
    search_Depth++;
    ROS_DEBUG_STREAM("Search_depth " << search_Depth);
  }
}

bool Router::route(DistLaneletList ego_lls) {
  if (currDest().first.empty()) {
    ROS_WARN("Routing Failed: Empty Destination List");
    return false;
  }
  if (ego_lls.empty()) {
    ROS_WARN("Routing Failed: Empty Ego Lanelets");
    return false;
  }

  // *** Find the best way to the destination(s) ***
  lanelet::Optional<lanelet::routing::LaneletPath> shortest_path;
  double shortest_path_length = INT_MAX;
  for (DistLanelet &start : ego_lls) {
    for (lanelet::ConstLanelet &dest : currDest().first) {
      auto path = routing_graph_->shortestPath(start.second, dest);
      if (!path) {
        continue;
      }
      double path_length = 0;
      for (const auto &ll : *path) {
        path_length += lanelet::geometry::length2d(ll);
      }
      if (!shortest_path || path_length < shortest_path_length) {
        shortest_path = path;
        shortest_path_length = path_length;
      }
    }
  }
  if (!shortest_path) {
    ROS_WARN_STREAM("Routing Failed: Unable to find path from ego set ["
                    << utils::extractIds(ego_lls) << "] to dest set ["
                    << utils::extractIds(currDest().first) << "]");
    return false;
  }
  route_ = shortest_path.get();
  for (unsigned int ll_i = 0; ll_i < route_.size(); ll_i++) {
    auto &ll = route_[ll_i];
    bool ll_is_last = true;
    for (unsigned int ll_next_i = ll_i + 1; ll_next_i < route_.size();
         ll_next_i++) {
      if (!laneChange(ll, route_[ll_next_i])) {
        ll_is_last = false;
        break;
      }
    }
    if (ll_is_last) {
      route_end_.first = ll;
      route_end_.second = lanelet::geometry::toArcCoordinates(ll.centerline2d(),
                                                              currDest().second)
                              .length;
    }
  }
  ROS_INFO_STREAM("Found path to destination with " << route_.size()
                                                    << " lanelets");
  return true;
}

// Given a Point3d location, returns the closest lanelet on curr_path to that
// point
int Router::closestLaneletOnRoute(const lanelet::Point3d &ego_location,
                                  bool take_lane_change) {
  double minDistance = DBL_MAX;
  int retVal = -1;
  for (int i = 0; i < route_.size(); i++) {
    auto ll = route_[i];
    double curr_distance =
        lanelet::geometry::distanceToCenterline3d(ll, ego_location);
    if (curr_distance < minDistance) {
      minDistance = curr_distance;
      retVal = i;
    }
  }
  if (!take_lane_change) return retVal;
  while (retVal < route_.size() - 1 &&
         laneChange(route_[retVal], route_[retVal + 1]))
    retVal++;
  return retVal;
}

// Based on getRemainingLane() for LaneletPath
// Given constlanelet on curr_path, return remaining lanelet sequence of
// curr_path, either to destination, or next set of traffic lights
lanelet::LaneletSequence Router::routeToFirstTrafficLight(int llt) {
  lanelet::ConstLanelets lane;
  while (llt < route_.size()) {
    lane.push_back(route_[llt]);
    if (route_[llt].regulatoryElementsAs<lanelet::TrafficLight>().size() > 0) {
      break;
    }
    llt++;
  }
  return lane;
}

lanelet::routing::LaneletPath Router::getRoute() { return route_; }

bool Router::laneChange(const lanelet::ConstLanelet &from,
                        const lanelet::ConstLanelet &to) {
  auto besides_from = routing_graph_->besides(from);
  return std::find(besides_from.begin(), besides_from.end(), to) !=
         besides_from.end();
}

std::pair<lanelet::ConstLanelet, double> Router::routeEnd() {
  return route_end_;
}

DestinationList Router::getDesinations() { return dest_list_; }