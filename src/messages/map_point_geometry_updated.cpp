//
// Created by vahagn on 13.08.21.
//

#include "map_point_geometry_updated.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

MapPointGeometryUpdated::MapPointGeometryUpdated(const map::MapPoint * map_point)
    : position(map_point->GetPosition()), id((size_t) map_point) {

}

MapPointGeometryUpdated::MapPointGeometryUpdated(const std::vector<uint8_t> & serialized) {
  INIT_DESERIALIZATION(serialized);
  COPY_FROM(source, id);
  DeSerializePoint(source, position);
}

MessageType MapPointGeometryUpdated::Type() const {
  return MAP_POINT_GEOMETRY_UPDATED;
}

void MapPointGeometryUpdated::Serialize(std::vector<uint8_t> & out_serialized) const {
  INIT_SERIALIZATION(out_serialized, sizeof(id) + POINT_SIZE);
  COPY_TO(dest, id);
  SerializePoint(position, dest);
}

}
}