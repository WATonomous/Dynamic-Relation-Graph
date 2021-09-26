#include "pp_env_model/behavioral/stop_result.hpp"

using namespace path_planning;
using namespace path_planning::behavioral;

void DynamicObjectStopResult::testStop(const geom::Polyline &ref_line, const VehicleState &ego_pose, const Object& obj) {
    if (ref_line.empty()) {
        _has_stop = false;
        return;
    }
    
    unsigned int egoIndex = ref_line.closestElementIndex(ego_pose);
    unsigned int objIndex = ref_line.closestElementIndex(obj.center);

    double egoDistToObj = ref_line.distanceBetween(egoIndex, objIndex);
    double refDistToObj = obj.distanceTo(ref_line[objIndex]);
    if (egoDistToObj < 0 || refDistToObj > Dynamic_Config.groups.behavioral_planning.OBJECT_ROI_RADIUS) {
        _has_stop = false;
        return;
    }

    double walkBackDist = 0;
    while (objIndex >= 1 && walkBackDist < Dynamic_Config.groups.behavioral_planning.OBJECT_STOPPING_DIST) {
        walkBackDist += geom::LineSegment(ref_line[objIndex], ref_line[objIndex - 1]).length();
        objIndex--;
    }
    _has_stop = true;
    _stop_point = ref_line[objIndex];
    _stopping_dist = ref_line.distanceBetween(0, objIndex);
    _obj = obj;
    _ref_line = ref_line;
}

DynamicObjectStopResult::DynamicObjectStopResult(const geom::Polyline &ref_line, const VehicleState &ego_pose, const Object& obj) {
    testStop(ref_line, ego_pose, obj);
}

void DynamicObjectStopResult::updateDistance(const geom::Polyline &ref_line, const VehicleState &ego_pose, const Object& obj) {
    if (ref_line[0] == _ref_line[0] && obj.center == _obj.center) {
        // None of the information we base our measurements on has changed
        return;
    }

    testStop(ref_line, ego_pose, obj);
}

void TrafficRegulationStopResult::testStop(const Lane& current_lane) {
    if (current_lane.empty()) {
        _has_stop = false;
        return;
    }

    // TODO: check if lane is stopping lane, for now assume all lanes are stopping lanes
    _has_stop = true;
    _stop_point = current_lane.back();
    _stopping_dist = current_lane.length();
    _current_lane = current_lane;
}

TrafficRegulationStopResult::TrafficRegulationStopResult(const Lane& current_lane) {
    testStop(current_lane);
}

void TrafficRegulationStopResult::updateDistance(const Lane& current_lane) {
    if (current_lane[0] == _current_lane[0]) {
        // None of the information we base our measurements on has changed
        return;
    }
    testStop(current_lane);
}

bool StopResult::hasStop() const {
    return _has_stop;
}

geom::Point2d StopResult::getStopPoint() const {
    assert(_has_stop);

    return _stop_point;
}

double StopResult::getDistance() const {
    assert(_has_stop);

    return _stopping_dist;
}

StopResult::StopResult(): _has_stop{false} {}

DynamicObjectStopResult::DynamicObjectStopResult(): StopResult() {}

TrafficRegulationStopResult::TrafficRegulationStopResult(): StopResult() {}