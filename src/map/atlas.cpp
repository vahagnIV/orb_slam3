//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "atlas.h"
#include <settings.h>
#include <messages/messages.h>
#include <serialization/serialization_context.h>

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
  maps_.insert(map);
  current_map_ = map;
}

size_t Atlas::GetMapCount() const {
  return maps_.size();
}

const std::unordered_set<map::Map *> &Atlas::GetMaps() const {
  return maps_;
}

void Atlas::Serialize(std::ostream &ostream) const {

  std::unordered_set<const camera::ICamera *> cameras;
  for (const auto map: maps_) {
    for (auto kf: map->GetAllKeyFrames()) {
      cameras.insert(kf->GetCamera());
    }
  }

  size_t camera_count = cameras.size();
  WRITE_TO_STREAM(camera_count, ostream);
  for (auto camera: cameras) {
    camera->Serialize(ostream);
  }

  size_t map_count = GetMapCount();
  WRITE_TO_STREAM(map_count, ostream);

  for (const auto map: maps_) {
    map->Serialize(ostream);
  }

}

void Atlas::Deserialize(std::istream &istream) {
//  context.map_id;
}

}
}