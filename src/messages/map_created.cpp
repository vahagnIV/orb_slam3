//
// Created by vahagn on 13/08/2021.
//

#include "map_created.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

MapCreated::MapCreated(map::Map * map) : map((size_t) map) {

}

MapCreated::MapCreated(const std::vector<uint8_t> & serialized) {
  INIT_DESERIALIZATION(serialized);
  COPY_FROM(source, map);
}

void MapCreated::Serialize(std::vector<uint8_t> & out_serialized) const {
  INIT_SERIALIZATION(out_serialized, sizeof(map));
  COPY_TO(dest, map);
}

MessageType MapCreated::Type() const {
  return MAP_CREATED;
}

}
}