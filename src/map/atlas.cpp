//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "atlas.h"
#include <settings.h>
#include <messages/messages.h>

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
  if (Settings::Get().MessageRequested(messages::MAP_CREATED))
    messages::MessageProcessor::Instance().Enqueue(new messages::MapCreated(current_map_));
  maps_.insert(current_map_);
}

Atlas::~Atlas() {

}
void Atlas::SetCurrentMap(map::Map * map) {
  current_map_ = map;
}

size_t Atlas::GetMapCount() const {
  return maps_.size();
}

const std::unordered_set<map::Map *> & Atlas::GetMaps() const {
  return maps_;
}

}
}