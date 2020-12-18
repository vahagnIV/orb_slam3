//
// Created by vahagn on 12/8/20.
//

#include "atlas.h"

namespace nvision {

Atlas::Atlas() : current_map_(nullptr) {

}

Map *Atlas::GetCurrentMap() const{
  if(nullptr == current_map_)

  return current_map_;
}

void Atlas::CreateNewMap() {
  current_map_ = new Map();
}

Atlas::~Atlas() {

}

}