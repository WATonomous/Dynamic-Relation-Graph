#pragma once

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <ros/ros.h>

#include <unordered_set>

#include "common_msgs/TrafficSignList.h"
#include "path_planning_msgs/DestinationList.h"
#include "pp_env_model/env_model_utils.hpp"

namespace path_planning {
namespace env_model {

class Router {
 public:
  Router(lanelet::LaneletMapPtr map);
  void build(lanelet::LaneletMapPtr map);
  void setDesinations(DestinationList dests);
  DestinationList getDesinations();
  lanelet::routing::RoutingGraphPtr graph();
  void seachForTurnDir(lanelet::ConstLanelet &adjacent,
                       ConstLaneletSet &turnDirCandidateLLs);
  bool route(DistLaneletList ego_lls);
  lanelet::routing::LaneletPath getRoute();
  int closestLaneletOnRoute(const lanelet::Point3d &ego_location,
                            bool take_lane_change = false);
  lanelet::LaneletSequence routeToFirstTrafficLight(int llt);
  std::pair<lanelet::ConstLanelet, double> routeEnd();
  bool laneChange(const lanelet::ConstLanelet &from,
                  const lanelet::ConstLanelet &to);

 private:
  int kMaxSearchDepthForIntersectionLlBFS = 2;

  Destination currDest();

  lanelet::routing::RoutingGraphPtr routing_graph_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  DestinationList dest_list_;
  lanelet::routing::LaneletPath route_;
  std::pair<lanelet::ConstLanelet, double> route_end_;
};

}  // namespace env_model
}  // namespace path_planning
