//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_MAP_POINT_DELETED_H_
#define ORB_SLAM3_SRC_MESSAGES_MAP_POINT_DELETED_H_
#include "base_message.h"
#include <map/map_point.h>

namespace orb_slam3 {
namespace messages {

class MapPointDeleted : public BaseMessage{
 public:
  MapPointDeleted(const map::MapPoint * map_point);
  MessageType Type() const override;
  size_t id;

};

}
}
#endif //ORB_SLAM3_SRC_MESSAGES_MAP_POINT_DELETED_H_
