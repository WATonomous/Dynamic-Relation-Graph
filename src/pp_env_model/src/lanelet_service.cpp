#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>
#include <sensor_msgs/NavSatFix.h>

#include <experimental/filesystem>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "autoware_lanelet2_msgs/MapBin.h"
#include "lanelet2_extension/utility/message_conversion.h"
#include "path_planning_msgs/Lanelet2MapName.h"
#include "path_planning_msgs/Lanelet2MapService.h"
#include "ros/ros.h"

using namespace lanelet;
using namespace path_planning_msgs;

struct GPSBounds {
  explicit GPSBounds(const GPSPoint &min, const GPSPoint &max)
      : min{min}, max{max} {}
  GPSPoint min;
  GPSPoint max;
  bool within(const GPSPoint &q) const {
    return q.lat >= min.lat && q.lon >= min.lon && q.lat <= max.lat &&
           q.lon <= max.lon;
  }
};

class SphericalMercatorProjectorWithOffset : public lanelet::Projector {
 private:
  projection::SphericalMercatorProjector sphericalMercatorProjector;
  BasicPoint3d originBasicPoint3d;

 public:
  SphericalMercatorProjectorWithOffset(Origin origin)
      : lanelet::Projector{origin},
        sphericalMercatorProjector{origin},
        originBasicPoint3d{
            sphericalMercatorProjector.forward(origin.position)} {}
  BasicPoint3d forward(const GPSPoint &p) const override {
    return sphericalMercatorProjector.forward(p) - originBasicPoint3d;
  }
  GPSPoint reverse(const BasicPoint3d &p) const override {
    return sphericalMercatorProjector.reverse(p + originBasicPoint3d);
  }
};

std::vector<std::pair<GPSBounds, std::string>> GPS_BOUNDS_TO_MAP_NAME = {
    std::make_pair(GPSBounds(GPSPoint{43.4728308, -80.5408251},
                             GPSPoint{43.474824, -80.5375421}),
                   "avril"),
    std::make_pair(GPSBounds(GPSPoint{43.4657374, -80.5463076},
                             GPSPoint{43.4752366, -80.5413938}),
                   "ringroad"),
    std::make_pair(GPSBounds(GPSPoint{43.4657374, -80.5463076},
                             GPSPoint{43.4729476, -80.5386043}),
                   "ringroad"),
    std::make_pair(GPSBounds(GPSPoint{43.4327979, -80.5823243},
                             GPSPoint{43.4379943, -80.5759728}),
                   "firetower"),
    std::make_pair(GPSBounds(GPSPoint{43.498923, -80.5505695},
                             GPSPoint{43.5016862, -80.5454874}),
                   "clearpath"),
    std::make_pair(GPSBounds(GPSPoint{42.2, -83.7}, GPSPoint{43.3, -83.6}),
                   "mcity"),
};

std::string MAPS_DIR;

auto MAP_PROJECTORS =
    std::unordered_map<std::string, std::shared_ptr<lanelet::Projector>>();

auto MAP_FILENAMES = std::unordered_map<std::string, std::string>(
    {{"mcity_mathworks", "mcity"}});

void populateMapProjectors() {
  MAP_PROJECTORS["mcity"] =
      std::make_shared<SphericalMercatorProjectorWithOffset>(
          Origin(GPSPoint{42.2998, -83.6989}));
  MAP_PROJECTORS["mcity_mathworks"] =
      std::make_shared<SphericalMercatorProjectorWithOffset>(
          Origin(GPSPoint{42.299848, -83.698862}));
}

lanelet::Optional<GPSPoint> try_get_origin_from_rosparam() {
  double origin_lat;
  double origin_lon;
  double origin_ele = 0;
  if (!ros::param::get("/map_origin/lat", origin_lat) ||
      !ros::param::get("/map_origin/lon", origin_lon)) {
    return {};
  }
  ros::param::get("/map_origin/ele", origin_ele);
  GPSPoint ret{origin_lat, origin_lon, origin_ele};
  return ret;
}

std::unordered_set<std::string> get_maps_from_origin_params() {
  lanelet::Optional<GPSPoint> q = try_get_origin_from_rosparam();
  if (!q) {
    return {};
  }
  std::unordered_set<std::string> in_maps;
  for (const auto &p : GPS_BOUNDS_TO_MAP_NAME) {
    if (p.first.within(q.get())) {
      in_maps.insert(p.second);
    }
  }
  return in_maps;
}

std::string get_map_name(Lanelet2MapService::Request &req) {
  std::string carla_town;
  if (req.use_map_name.data) {
    return req.map_name.data;
  } else if (ros::param::get("/carla/town", carla_town)) {
    return carla_town;
  } else {
    std::unordered_set<std::string> map_names = get_maps_from_origin_params();
    if (map_names.empty()) return "";
    if (map_names.size() > 1) {
      ROS_ERROR("World location matches multiple maps");
      return "";
    }
    return *map_names.begin();
  }

  return "";
}

std::shared_ptr<lanelet::Projector> get_projector(
    const Lanelet2MapService::Request &req, const std::string &map_name) {
  // Origin from request with default projector
  if (req.use_origin.data) {
    ROS_INFO_STREAM(
        "Using default UTM projector with origin from request on map "
        << map_name);
    return std::make_shared<projection::UtmProjector>(
        Origin(GPSPoint{req.utm_origin.latitude, req.utm_origin.longitude}));
  }
  // Origin and projector from MAP_PROJECTORS
  else if (MAP_PROJECTORS.find(map_name) != MAP_PROJECTORS.end()) {
    ROS_INFO_STREAM("Using custom projector with origin on map " << map_name);
    return MAP_PROJECTORS.at(map_name);
  }
  // Origin from rosparam with default projector
  else {
    lanelet::Optional<GPSPoint> origin = try_get_origin_from_rosparam();
    if (!origin) {
      return nullptr;
    }
    ROS_INFO_STREAM(
        "Using default UTM projector with origin from rosparam on map "
        << map_name);
    return std::make_shared<projection::UtmProjector>(Origin(origin.get()));
  }
}

void transform_tl(lanelet::TrafficLight::Ptr tl_ptr, double ele) {
  for (auto &tl : tl_ptr->trafficLights()) {
    auto ls = tl.lineString();
    if (!ls) {
      return;
    }
    for (auto &p : ls.get()) {
      p.z() -= ele;
    }
  }
}

bool get_map_name(Lanelet2MapName::Request &req,
                  Lanelet2MapName::Response &res) {
  Lanelet2MapService::Request empty_req;
  empty_req.use_map_name.data = false;
  std::string map_name = get_map_name(empty_req);
  if (map_name.empty()) {
    ROS_ERROR_THROTTLE(5,
                       "Unable to determine Map to serve, please specify one");
    return false;
  }
  res.map_name.data = map_name;
  return true;
}

void originCallback(const sensor_msgs::NavSatFix &msg) {
  ros::param::set("/map_origin/lat", msg.latitude);
  ros::param::set("/map_origin/lon", msg.longitude);
  ros::param::set("/map_origin/ele", msg.altitude);
}

bool get_map(Lanelet2MapService::Request &req,
             Lanelet2MapService::Response &res) {
  std::string map_name = get_map_name(req);
  std::string map_filename;
  if (MAP_FILENAMES.find(map_name) != MAP_FILENAMES.end()) {
    map_filename = MAP_FILENAMES[map_name];
  } else {
    map_filename = map_name;
  }

  if (map_name.empty()) {
    ROS_ERROR_THROTTLE(5,
                       "Unable to determine Map to serve, please specify one");
    return false;
  }
  std::string map_file = MAPS_DIR + "/osm/" + map_filename + ".osm";
  ROS_INFO_STREAM("Serving " << map_file << " ...");
  if (!std::experimental::filesystem::exists(map_file)) {
    ROS_ERROR_STREAM(
        "Map "
        << map_file.c_str()
        << " does not exist. Please add it to "
           "https://git.uwaterloo.ca/WATonomous/map_data/-/tree/master/osm");
    return false;
  }

  std::shared_ptr<lanelet::Projector> projector = get_projector(req, map_name);
  if (!projector) {
    ROS_ERROR_THROTTLE(5, "Unable to determine projector to use");
    return false;
  }

  try {
    GPSPoint origin = projector->origin().position;
    ROS_INFO_STREAM("Loading " << map_name << " with origin ("
                               << std::setprecision(12) << origin.lat << ","
                               << origin.lon << ") ...");

    lanelet::LaneletMapPtr map = load(map_file, *projector);
    for (auto &reg : map->regulatoryElementLayer) {
      auto tl = std::dynamic_pointer_cast<TrafficLight>(reg);
      if (tl) {
        transform_tl(tl, origin.ele);
      }
    }
    ROS_INFO_STREAM("Point "
                    << *map->laneletLayer.begin()->centerline3d().begin());
    autoware_lanelet2_msgs::MapBin map_msg;
    lanelet::utils::conversion::toBinMsg(map, &map_msg);
    res.map_bin = map_msg;
    ROS_INFO("Success");
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM_THROTTLE(
        5, "Failed to load and convert map, error: " << e.what());
    return false;
  }
}

int main(int argc, char **argv) {
  populateMapProjectors();
  ros::init(argc, argv, "lanelet2_map_service");
  ros::NodeHandle n;
  if (!n.getParam("MAPS_DIR", MAPS_DIR)) {
    ROS_ERROR_THROTTLE(5, "MAPS_DIR param required");
    return 1;
  }
  if (!std::experimental::filesystem::exists(MAPS_DIR)) {
    ROS_ERROR_STREAM_THROTTLE(5, "MAPS_DIR " << MAPS_DIR << " does not exist");
    return 1;
  }

  auto map_service = n.advertiseService("/lanelet2_map_service", get_map);

  auto name_service = n.advertiseService("/lanelet2_map_name", get_map_name);

  ros::Subscriber origin = n.subscribe("/map_origin", 10, originCallback);

  ros::spin();

  return 0;
}
