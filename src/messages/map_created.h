//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_MAP_CREATED_H_
#define ORB_SLAM3_SRC_MESSAGES_MAP_CREATED_H_

#include "base_message.h"
#include <map/map.h>

namespace orb_slam3 {
namespace messages {

class MapCreated : public BaseMessage {
 public:
  MapCreated(map::Map * map);
  MessageType Type() const override;
  const size_t map;

};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_MAP_CREATED_H_
