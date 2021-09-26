#pragma once

#include "pp_common/point2d.hpp"
#include "pp_common/polyline.hpp"
#include "pp_common/vehicle_state.hpp"
#include "pp_common/line_segment.hpp"
#include "pp_common/object.hpp"
#include "pp_common/lane.hpp"
#include <pp_parameter_server/parameter_config.h>

namespace path_planning { namespace behavioral {

    class StopResult{
        public:
            StopResult();

            bool hasStop() const;
            geom::Point2d getStopPoint() const;
            double getDistance() const;

        protected:
            bool _has_stop = false;
            geom::Point2d _stop_point;
            double _stopping_dist;
    }; 

    class DynamicObjectStopResult: public StopResult {
        public:
            DynamicObjectStopResult();

            DynamicObjectStopResult(const geom::Polyline &ref_line, const VehicleState &ego_pose, const Object& obj);
            void updateDistance(const geom::Polyline &ref_line, const VehicleState &ego_pose, const Object& obj);

        private:
            void testStop(const geom::Polyline &ref_line, const VehicleState &ego_pose, const Object& obj);
            Object _obj;
            geom::Polyline _ref_line;
    };

    class TrafficRegulationStopResult: public StopResult {
        public:
            TrafficRegulationStopResult();

            TrafficRegulationStopResult(const Lane& current_lane);
            void updateDistance(const Lane& current_lane);
        private:
            void testStop(const Lane& current_lane);
            Lane _current_lane;
    };

}}
