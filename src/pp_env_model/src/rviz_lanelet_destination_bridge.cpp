#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/GenericTrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>

#include "lanelet2_extension/utility/message_conversion.h"
#include "path_planning_msgs/Lanelet2MapService.h"
#include "path_planning_msgs/LaneletDestinationService.h"

using namespace lanelet;
using namespace path_planning_msgs;

lanelet::LaneletMapPtr lanelet_map = std::make_shared<lanelet::LaneletMap>();
routing::RoutingGraphUPtr routingGraph;

bool get_destination(LaneletDestinationService::Request &req,
                     LaneletDestinationService::Response &res) {
  std::string dest = req.destination_address.data;
  double x, y;
  try {
    x = std::stod(dest.substr(0, dest.find(',')));
    y = std::stod(dest.substr(dest.find(',') + 1));
  } catch (std::exception e) {
    ROS_ERROR_STREAM("Invalid rviz address: " << dest
                                              << ". x,y format expected.");
    return false;
  }
  res.destination_point.x = x;
  res.destination_point.y = y;
  Point2d dest_point{utils::getId(), x, y};

  auto closest_lanelets =
      geometry::findWithin2d(lanelet_map->laneletLayer, dest_point, 10);
  if (closest_lanelets.size() == 0) {
    return false;
  }
  for (auto near_ll : closest_lanelets) {
    res.destination_lanelets.push_back(near_ll.second.id());
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "destination_address_service");
  ros::NodeHandle n;

  auto lanelet2_map_service =
      n.serviceClient<path_planning_msgs::Lanelet2MapService>(
          "/lanelet2_map_service");
  path_planning_msgs::Lanelet2MapService map_call;
  while (ros::ok() && !lanelet2_map_service.call(map_call)) {
    ros::Duration(1).sleep();
  }
  if (!ros::ok()) {
    return 0;
  }
  lanelet::utils::conversion::fromBinMsg(map_call.response.map_bin,
                                         lanelet_map);

  traffic_rules::TrafficRulesPtr trafficRules =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany,
                                                 Participants::Vehicle);
  routingGraph = routing::RoutingGraph::build(*lanelet_map, *trafficRules);

  ros::ServiceServer destination_service =
      n.advertiseService("/lanelet_destination_service", get_destination);
  std::cout << lanelet_map->laneletLayer.size() << std::endl;

  ros::spin();

  return 0;
}