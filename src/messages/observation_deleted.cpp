//
// Created by vahagn on 13/08/2021.
//

#include "observation_deleted.h"
#include <frame/key_frame.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace messages {
ObservationDeleted::ObservationDeleted(const frame::Observation & observation) :
    frame_id(observation.GetKeyFrame()->Id()), map_point_id((size_t) observation.GetMapPoint()) {

}

MessageType ObservationDeleted::Type() const {
  return OBSERVATION_DELETED;
}
}
}