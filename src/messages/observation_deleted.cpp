//
// Created by vahagn on 13/08/2021.
//

#include "observation_deleted.h"

namespace orb_slam3 {
namespace messages {
ObservationDeleted::ObservationDeleted(const frame::Observation & observation) {

}

MessageType ObservationDeleted::Type() const {
  return OBSERVATION_DELETED;
}
}
}