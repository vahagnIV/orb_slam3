//
// Created by vahagn on 13/08/2021.
//

#include "map_created.h"

namespace orb_slam3 {
namespace messages {

MapCreated::MapCreated(map::Map * map): map((size_t)map) {

}

MessageType MapCreated::Type() const {
  return MAP_CREATED;
}

}
}