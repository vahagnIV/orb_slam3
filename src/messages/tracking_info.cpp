//
// Created by vahagn on 13/08/2021.
//

#include "tracking_info.h"

namespace orb_slam3 {
namespace messages {
TrackingInfo::TrackingInfo(const geometry::Pose & pose,
                           const TimePoint time_point,
                           const geometry::Pose & velocity)
    : position(pose), time_point(time_point), velocity(velocity) {

}

MessageType TrackingInfo::Type() const {
  return TRACKING_INFO;
}
}
}