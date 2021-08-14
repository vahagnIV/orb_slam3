//
// Created by vahagn on 13/08/2021.
//

#include "observation_added.h"
#include <frame/key_frame.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace messages {
ObservationAdded::ObservationAdded(const frame::Observation & observation)
    : frame_id(observation.GetKeyFrame()->Id()), map_point_id((size_t) observation.GetMapPoint()) {

}

MessageType ObservationAdded::Type() const {
  return OBSERVATION_ADDED;
}

}
}