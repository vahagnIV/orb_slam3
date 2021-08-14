//
// Created by vahagn on 13/08/2021.
//

#include "map_point_deleted.h"

namespace orb_slam3 {
namespace messages {

MapPointDeleted::MapPointDeleted(const map::MapPoint * map_point) : id((size_t) map_point) {
}

MessageType MapPointDeleted::Type() const {
  return MAP_POINT_DELETED;
}

}
}