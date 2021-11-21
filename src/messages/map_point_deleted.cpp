//
// Created by vahagn on 13/08/2021.
//

#include "map_point_deleted.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

MapPointDeleted::MapPointDeleted(const map::MapPoint * map_point) : id((size_t) map_point) {
}

MapPointDeleted::MapPointDeleted(std::vector<uint8_t> & serialized) {
  INIT_DESERIALIZATION(serialized);
  COPY_FROM(source, id);
}

MessageType MapPointDeleted::Type() const {
  return MAP_POINT_DELETED;
}
void MapPointDeleted::Serialize(std::vector<uint8_t> & out_serialized) const {
  INIT_SERIALIZATION(out_serialized, sizeof(id));
  COPY_TO(dest, id);
}

}
}