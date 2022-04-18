//
// Created by vahagn on 18/04/2022.
//

#include "position_predicted.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

PositionPredicted::PositionPredicted(frame::BaseFrame * base_frame) : pose(base_frame->GetStagingPosition()) {

}

MessageType PositionPredicted::Type() const {
  return POSITION_PREDICTED;
}

void PositionPredicted::Serialize(std::vector<uint8_t> & out_serialized) const {
  INIT_SERIALIZATION(out_serialized, POSITION_SIZE);
  SerializePose(pose, dest);
}

}
}