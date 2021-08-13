//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_OBSERVATION_DELETED_H_
#define ORB_SLAM3_SRC_MESSAGES_OBSERVATION_DELETED_H_

#include "base_message.h"
#include <frame/observation.h>

namespace orb_slam3 {
namespace messages {

class ObservationDeleted : public BaseMessage {
 public:
  ObservationDeleted(const frame::Observation & observation);
  MessageType Type() const override;
  const size_t frame_id;
  const size_t map_point_id;
};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_OBSERVATION_DELETED_H_
