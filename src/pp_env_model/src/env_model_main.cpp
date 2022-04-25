#include <ros/console.h>
#include <signal.h>

#include "lanelet2_extension/utility/message_conversion.h"
#include "path_planning_msgs/Lanelet2MapService.h"
#include "path_planning_msgs/LaneletDestinationService.h"
#include "path_planning_msgs/ReferenceLine.h"
#include "pp_env_model/env_model.hpp"
#include "pp_env_model/wato_traffic_rules.hpp"

using namespace path_planning;

std::unique_ptr<env_model::EnvModel> env_model_ptr;
ros::Publisher tl_pub;
ros::Publisher ref_line_pub;
ros::Publisher viz_pub;
ros::Publisher map_viz_pub;
ros::Publisher route_viz_pub;
ros::ServiceClient lanelet_destination_service;

void mySigintHandler(int sig) {
  viz_pub.publish(env_model::utils::deleteAllMarker());
  map_viz_pub.publish(env_model::utils::deleteAllMarker());
  env_model_ptr->route_viz_pub.publish(env_model::utils::deleteAllMarker());
  ros::shutdown();
}

void destination_list_callback(
    path_planning_msgs::DestinationList dest_list_msg) {
  path_planning_msgs::LaneletDestinationService destination_call;
  env_model::DestinationList dest_list;
  for (const std::string &dest_name : dest_list_msg.destination_list) {
    destination_call.request.destination_address.data = dest_name;
    if (!lanelet_destination_service.call(destination_call) ||
        destination_call.response.destination_lanelets.empty()) {
      ROS_WARN_STREAM(
          "Routing Failed: Destination lanelet service failed for destination "
          "address: "
          << destination_call.request.destination_address.data);
      return;
    }
    env_model::Destination dest;
    for (const int &id : destination_call.response.destination_lanelets) {
      dest.first.push_back(*env_model_ptr->map()->laneletLayer.find(id));
    }
    dest.second = lanelet::Point2d{
        lanelet::InvalId, destination_call.response.destination_point.x,
        destination_call.response.destination_point.y};

    dest_list.push_back(dest);
  }
  env_model_ptr->router.setDesinations(dest_list);
  env_model_ptr->route();
}

void traffic_light_timer(const ros::TimerEvent &e) {
  auto tl = env_model_ptr->nextTrafficLight();
  if (!!tl) tl_pub.publish(tl.get());
}

void viz_regulatory_elements(const ros::TimerEvent &e) {
  for (const auto &ll : env_model_ptr->map()->laneletLayer) {
    visualization_msgs::MarkerArray ma = env_model::utils::regElemMarkers(ll);
    if (ma.markers.empty()) continue;
    viz_pub.publish(ma);
  }
}

void odom_callback(nav_msgs::Odometry odom) {
  env_model_ptr->updateStopSigns(odom);
  path_planning_msgs::ReferenceLine ref = env_model_ptr->localRef(
      env_model::utils::geomToPoint(odom.pose.pose.position));
  ref.header.frame_id = "odom";
  ref_line_pub.publish(ref);
  viz_pub.publish(env_model::utils::createRefLineMarker(ref.ref_line));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "env_model");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();
  ros::NodeHandle n;

  lanelet::RegisterRegulatoryElement<env_model::PedRegElem> ped_reg;
  lanelet::RegisterRegulatoryElement<env_model::StopSignRegElem> stop_sign_reg;
  lanelet::traffic_rules::RegisterTrafficRules<
      lanelet::traffic_rules::WatoVehicle>
      regMyClass("wato", lanelet::Participants::Vehicle);

  // *** Initialize Env Model ***
  auto lanelet2_map_service =
      n.serviceClient<path_planning_msgs::Lanelet2MapService>(
          "/lanelet2_map_service");
  path_planning_msgs::Lanelet2MapService map_call;
  while (ros::ok() && !lanelet2_map_service.call(map_call)) {
    ROS_INFO("Waiting for Lanelet2MapService");
    ros::Duration(1).sleep();
  }
  if (!ros::ok()) {
    return 0;
  }
  lanelet::LaneletMapPtr lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(map_call.response.map_bin,
                                         lanelet_map);
  env_model_ptr = std::make_unique<env_model::EnvModel>(lanelet_map, n);

  // Setup service clients
  lanelet_destination_service =
      n.serviceClient<path_planning_msgs::LaneletDestinationService>(
          "/lanelet_destination_service");
  // Setup subscribers
  ros::Subscriber tracked_intersection_signs_subscriber = n.subscribe(
      "/tracked_signs", 1, &env_model::EnvModel::trafficSignCallback,
      env_model_ptr.get());
  ros::Subscriber tracked_obstacle_subscriber = n.subscribe(
      "/tracked_obstacles", 1, &env_model::EnvModel::trackedObstacleCallback,
      env_model_ptr.get());
  ros::Subscriber detected_obtacle_subscriber =
      n.subscribe("/occupancy_map", 1, &env_model::EnvModel::occGridCallback,
                  env_model_ptr.get());
  ros::Subscriber traffic_light_detection_sub = n.subscribe(
      "/traffic_light_detection", 1,
      &env_model::EnvModel::trafficLightDetectionCallback, env_model_ptr.get());

  ros::Subscriber destination_list_sub = n.subscribe(
      "/path_planning/destination_list", 1, destination_list_callback);
  ros::Subscriber odom_sub = n.subscribe("/navsat/odom", 1, odom_callback);
  // Setup timers
  ros::Timer timer =
      n.createTimer(ros::Duration(1.f), &env_model::EnvModel::pruneDeadTracks,
                    env_model_ptr.get());
  ros::Timer tl_timer = n.createTimer(ros::Duration(0.5), traffic_light_timer);
  ros::Timer reg_elem_timer =
      n.createTimer(ros::Duration(0.01), viz_regulatory_elements);
  // Setup publishers
  tl_pub =
      n.advertise<common_msgs::TrafficLightList>("traffic_light_list", 100);
  ref_line_pub = n.advertise<path_planning_msgs::ReferenceLine>(
      "/path_planning/ref_line", 100);
  viz_pub = n.advertise<visualization_msgs::MarkerArray>(
      "/path_planning/env_model_viz", 100);
  map_viz_pub = n.advertise<visualization_msgs::MarkerArray>(
      "/path_planning/map_viz", 1, true);
  route_viz_pub = n.advertise<visualization_msgs::MarkerArray>(
      "/path_planning/route_viz", 1);

  map_viz_pub.publish(env_model::utils::mapMarkers(lanelet_map));

  signal(SIGINT, mySigintHandler);
  ROS_INFO("Spinning");
  ros::spin();
  return 0;
}
