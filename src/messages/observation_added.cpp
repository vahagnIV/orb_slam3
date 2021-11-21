//
// Created by vahagn on 13/08/2021.
//

#include "observation_added.h"
#include "serialization_utils.h"
#include <frame/key_frame.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace messages {
ObservationAdded::ObservationAdded(const frame::Observation & observation)
    : frame_id(observation.GetKeyFrame()->Id()), map_point_id((size_t) observation.GetMapPoint()) {

}

ObservationAdded::ObservationAdded(const std::vector<uint8_t> & serialized) {
  INIT_DESERIALIZATION(serialized);
  COPY_FROM(source, frame_id);
  COPY_FROM(source, map_point_id);
}

MessageType ObservationAdded::Type() const {
  return OBSERVATION_ADDED;
}

void ObservationAdded::Serialize(std::vector<uint8_t> & out_serialized) const {
  INIT_SERIALIZATION(out_serialized, sizeof(frame_id) + sizeof(map_point_id));
  COPY_TO(dest, frame_id);
  COPY_TO(dest, map_point_id);

}

}
}