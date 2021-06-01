//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map.h"

namespace orb_slam3 {
namespace map {

void Map::AddKeyFrame(frame::KeyFrame * frame) {
  key_frames_.insert(frame);
}

void Map::AddMapPoint(MapPoint * map_point) {
  map_points_.insert(map_point);
}

std::unordered_set<MapPoint *> Map::GetAllMapPoints() {
  return map_points_;
}

void Map::SetInitialKeyFrame(frame::KeyFrame * frame) {
  initial_keyframe_ = frame;
}

}
}