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
  MapPointCreated(const std::vector<uint8_t> & out_serialized);
  MessageType Type() const override;
  void Serialize(std::vector<uint8_t> & out_serialized) const override;
  size_t id;
  TPoint3D position;
  size_t map_id;

};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_MAP_POINT_CREATED_H_
