//
// Created by vahagn on 13.08.21.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_MAP_POINT_GEOMETRY_UPDATED_H_
#define ORB_SLAM3_SRC_MESSAGES_MAP_POINT_GEOMETRY_UPDATED_H_

#include "base_message.h"
#include <map/map_point.h>

namespace orb_slam3 {
namespace messages {

class MapPointGeometryUpdated : public BaseMessage {
 public:
  MapPointGeometryUpdated(const map::MapPoint * map_point);
  MapPointGeometryUpdated(const std::vector<uint8_t> & serialized);
  void Serialize(std::vector<uint8_t> & out_serialized) const override;
  MessageType Type() const override;
  TPoint3D position;
  size_t id;

};

}
}
#endif //ORB_SLAM3_SRC_MESSAGES_MAP_POINT_GEOMETRY_UPDATED_H_
