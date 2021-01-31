//
// Created by vahagn on 12/8/20.
//

#include "atlas.h"

namespace orb_slam3 {

Atlas::Atlas() : current_map_(nullptr) {

}

Map *Atlas::GetCurrentMap() {
  if(nullptr == current_map_)
    CreateNewMap();

  return current_map_;
}

void Atlas::CreateNewMap() {
  current_map_ = new Map();
}

Atlas::~Atlas() {

}

}