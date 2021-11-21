//
// Created by vahagn on 13/08/2021.
//

#include "tracking_info.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {
TrackingInfo::TrackingInfo(const geometry::Pose & pose,
                           const TimePoint time_point,
                           const geometry::Pose & velocity)
    : position(pose), time_point(time_point), velocity(velocity) {

}

TrackingInfo::TrackingInfo(const std::vector<uint8_t> & serialized) {
  INIT_DESERIALIZATION(serialized);
  size_t nano;
  COPY_FROM(source, nano);
  time_point = TimePoint(std::chrono::duration_cast<std::chrono::system_clock::duration>(
      std::chrono::nanoseconds(nano)));
}

MessageType TrackingInfo::Type() const {
  return TRACKING_INFO;
}

void TrackingInfo::Serialize(std::vector<uint8_t> & out_serialized) const {
  INIT_SERIALIZATION(out_serialized, POSITION_SIZE + POSITION_SIZE + sizeof(size_t));
  size_t re = std::chrono::duration_cast<std::chrono::nanoseconds>(time_point.time_since_epoch()).count();
  COPY_TO(dest, re);
  SerializePose(position, dest);
  SerializePose(velocity, dest);
}

}
}