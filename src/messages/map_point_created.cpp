//
// Created by vahagn on 13/08/2021.
//

#include "map_point_created.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

MapPointCreated::MapPointCreated(const map::MapPoint * map_point)
    : id((size_t) map_point), position(map_point->GetPosition()), map_id((size_t) map_point->GetMap()) {

}

MapPointCreated::MapPointCreated(const std::vector<uint8_t> & serialized) {
  INIT_DESERIALIZATION(serialized);
  COPY_FROM(source, id);
  COPY_FROM(source, map_id);
  DeSerializePoint(source, position);
}

MessageType MapPointCreated::Type() const {
  return MAP_POINT_CREATED;
}

void MapPointCreated::Serialize(std::vector<uint8_t> & out_serialized) const {
  INIT_SERIALIZATION(out_serialized, POINT_SIZE + sizeof(id) + sizeof(map_id));
  COPY_TO(dest, id);
  COPY_TO(dest, map_id);
  SerializePoint(position, dest);

}

}
}