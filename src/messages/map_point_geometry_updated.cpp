//
// Created by vahagn on 13.08.21.
//

#include "map_point_geometry_updated.h"

namespace orb_slam3 {
namespace messages {

MapPointGeometryUpdated::MapPointGeometryUpdated(const map::MapPoint * map_point) : position(map_point->GetPosition()), id((size_t)map_point) {

}

MessageType MapPointGeometryUpdated::Type() const {
  return MAP_POINT_GEOMETRY_UPDATED;
}

}
}