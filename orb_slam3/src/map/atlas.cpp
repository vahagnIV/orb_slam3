//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map/atlas.h"

namespace orb_slam3 {
namespace map {
Atlas::Atlas() : current_map_(nullptr) {

}

Map * Atlas::GetCurrentMap() {
  if (nullptr == current_map_)
    CreateNewMap();

  return current_map_;
}

void Atlas::CreateNewMap() {
  current_map_ = new Map();
}

Atlas::~Atlas() {

}
}
}