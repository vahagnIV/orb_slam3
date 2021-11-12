//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_MAP_CREATED_H_
#define ORB_SLAM3_SRC_MESSAGES_MAP_CREATED_H_

#include <vector>

#include "base_message.h"
#include <map/map.h>

namespace orb_slam3 {
namespace messages {

class MapCreated : public BaseMessage {
 public:
  MapCreated(map::Map * map);
  MapCreated(const std::vector<uint8_t> & serialized);
  void Serialize(std::vector<uint8_t> & out_serialized) const override;

  MessageType Type() const override;
  size_t map;

};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_MAP_CREATED_H_
