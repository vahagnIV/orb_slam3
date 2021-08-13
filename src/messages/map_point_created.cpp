//
// Created by vahagn on 13/08/2021.
//

#include "map_point_created.h"

namespace orb_slam3 {
namespace messages {

MapPointCreated::MapPointCreated(const map::MapPoint * map_point)
    : id((size_t) map_point), position(map_point->GetPosition()), map_id((size_t) map_point->GetMap()) {

}

MessageType MapPointCreated::Type() const {
  return MAP_POINT_CREATED;
}

}
}