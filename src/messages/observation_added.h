//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_OBSERVATION_ADDED_H_
#define ORB_SLAM3_SRC_MESSAGES_OBSERVATION_ADDED_H_

#include "base_message.h"
#include <frame/observation.h>

namespace orb_slam3 {
namespace messages {

class ObservationAdded : public BaseMessage {
 public:
  ObservationAdded(const frame::Observation & observation);
  ObservationAdded(const std::vector<uint8_t> & serialized);
  void Serialize(std::vector<uint8_t> & out_serialized) const override;
  MessageType Type() const override;
  size_t frame_id;
  size_t map_point_id;

};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_OBSERVATION_ADDED_H_
