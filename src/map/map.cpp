//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map.h"
#include <frame/key_frame.h>
#define WRITE_TO_STREAM(num, stream) stream.write((char *)(&num), sizeof(num));
namespace orb_slam3 {
namespace map {

void Map::AddKeyFrame(frame::KeyFrame * frame) {
  key_frames_.insert(frame);
}

void Map::AddMapPoint(MapPoint * map_point) {
  map_points_.insert(map_point);
}

std::unordered_set<MapPoint *> Map::GetAllMapPoints() const {
  std::unordered_set<MapPoint *> result;
  for (auto mp: map_points_)
    if (!mp->IsBad())
      result.insert(mp);
  return result;
}

void Map::SetInitialKeyFrame(frame::KeyFrame * frame) {
  initial_keyframe_ = frame;
  key_frames_.insert(frame);
}

std::unordered_set<frame::KeyFrame *> Map::GetAllKeyFrames() const {
  std::unordered_set<frame::KeyFrame *> result;
  for (auto kf: key_frames_)
    if (!kf->IsBad())
      result.insert(kf);
  return result;
}

std::ostream & operator<<(ostream & stream, const Map * map) {
  size_t kf_count = map->key_frames_.size();
  WRITE_TO_STREAM(kf_count, stream);
  for (auto kf: map->key_frames_)
    stream << kf;

  size_t mp_count = map->map_points_.size();
  WRITE_TO_STREAM(mp_count, stream);

  for (auto mp: map->map_points_)
    stream << mp;



  return stream;
}

}
}
