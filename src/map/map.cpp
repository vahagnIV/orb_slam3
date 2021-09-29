//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map.h"
#include <frame/key_frame.h>
#include <map/map_point.h>
#include <serialization/serialization_context.h>
#include <factories/key_frame_factory.h>

namespace orb_slam3 {
namespace map {

void Map::AddKeyFrame(frame::KeyFrame * key_frame) {
  key_frames_.insert(key_frame);
}

void Map::EraseKeyFrame(frame::KeyFrame * key_frame) {
  key_frames_.erase(key_frame);
}

void Map::AddMapPoint(MapPoint * map_point) {
  map_points_.insert(map_point);
}

void Map::EraseMapPoint(MapPoint * map_point) {
  map_points_.erase(map_point);
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

std::ostream & operator<<(std::ostream & stream, const Map * map) {
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

void Map::Serialize(std::ostream & ostream) const {

  size_t kf_count = GetAllKeyFrames().size();
  WRITE_TO_STREAM(kf_count, ostream);
  for (const auto kf: GetAllKeyFrames()) {
    size_t kf_type = kf->Type();
    WRITE_TO_STREAM(kf_type, ostream);
    kf->Serialize(ostream);
  }

  size_t mp_count = GetAllMapPoints().size();
  WRITE_TO_STREAM(mp_count, ostream);
  for (const auto mp: GetAllMapPoints()) {
    mp->Serialize(ostream);
    MapPoint::MapType observations = mp->Observations();
    size_t observation_size = observations.size();
    WRITE_TO_STREAM(observation_size, ostream);
    for (auto obs: observations) {
      obs.second.Serialize(ostream);
    }
  }

}

void Map::Deserialize(std::istream & istream, serialization::SerializationContext & context) {
  size_t kf_count;
  READ_FROM_STREAM(kf_count, istream);
  for (size_t i = 0; i < kf_count; ++i) {
    frame::FrameType kf_type;
    READ_FROM_STREAM(kf_type, istream);
    auto kf = factories::KeyFrameFactory::Create(kf_type);
    kf->Deserialize(istream, context);
  }

}

}
}
