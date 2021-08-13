//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_MAP_POINT_CREATED_H_
#define ORB_SLAM3_SRC_MESSAGES_MAP_POINT_CREATED_H_
#include "base_message.h"
#include <geometry/pose.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace messages {

class MapPointCreated : public BaseMessage {
 public:
  MapPointCreated(const map::MapPoint * map_point);
  MessageType Type() const override;
  const size_t id;
  const TPoint3D position;
  const size_t map_id;

};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_MAP_POINT_CREATED_H_
