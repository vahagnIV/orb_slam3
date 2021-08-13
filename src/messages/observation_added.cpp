//
// Created by vahagn on 13/08/2021.
//

#include "observation_added.h"

namespace orb_slam3 {
namespace messages {
ObservationAdded::ObservationAdded(const frame::Observation & observation) {

}

MessageType ObservationAdded::Type() const {
  return OBSERVATION_ADDED;
}

}
}