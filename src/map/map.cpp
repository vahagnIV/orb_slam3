//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map.h"
#include "atlas.h"
#include <frame/key_frame.h>
#include <map/map_point.h>
#include <serialization/serialization_context.h>
#include <factories/key_frame_factory.h>

namespace orb_slam3 {
namespace map {

Map::Map(Atlas *atlas) : atlas_(atlas) {

}

void Map::AddKeyFrame(frame::KeyFrame *key_frame) {
  key_frames_.insert(key_frame);
}

void Map::EraseKeyFrame(frame::KeyFrame *key_frame) {
  key_frames_.erase(key_frame);
}

void Map::AddMapPoint(MapPoint *map_point) {
  std::unique_lock<std::mutex> lock(map_points_mutex_);
  map_points_.insert(map_point);
}

void Map::EraseMapPoint(MapPoint * map_point) {
  map_points_.erase(map_point);
}

std::unordered_set<MapPoint *> Map::GetAllMapPoints() const {
  std::unique_lock<std::mutex> lock(map_points_mutex_);
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
    frame::FrameType kf_type = kf->Type();
    WRITE_TO_STREAM(kf_type, ostream);
    kf->Serialize(ostream);
  }

  size_t mp_count = GetAllMapPoints().size();
  WRITE_TO_STREAM(mp_count, ostream);
  for (const auto mp: GetAllMapPoints()) {
    size_t mp_id = reinterpret_cast<size_t>(mp);
    WRITE_TO_STREAM(mp_id, ostream);
    mp->Serialize(ostream);
  }

  for (const auto mp: GetAllMapPoints()) {
    MapPoint::MapType observations = mp->Observations();
    size_t observation_size = observations.size();
    WRITE_TO_STREAM(observation_size, ostream);
    for (auto & obs: observations) {
      assert(map_points_.find(obs.second.GetMapPoint()) != map_points_.end());
      assert(obs.second.GetKeyFrame() == obs.first);
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
    auto kf = factories::KeyFrameFactory::Create(kf_type, istream, context);
    context.kf_id[kf->Id()] = kf;
    AddKeyFrame(kf);
  }

  size_t mp_count;
  READ_FROM_STREAM(mp_count, istream);

  for (size_t i = 0; i < mp_count; ++i) {
    size_t mp_id;
    READ_FROM_STREAM(mp_id, istream);
    auto mp = new map::MapPoint(istream, context);
    context.mp_id[mp_id] = mp;
    AddMapPoint(mp);
  }

  for (size_t i = 0; i < mp_count; ++i) {
    size_t observation_size;
    READ_FROM_STREAM(observation_size, istream);

    for (size_t j = 0; j < observation_size; ++j) {
      frame::Observation observation(istream, context);
      observation.GetKeyFrame()->AddMapPoint(observation);
    }
  }

  for (auto mp: GetAllMapPoints()) {
    mp->ComputeDistinctiveDescriptor();
    mp->CalculateNormalStaging();
    mp->ApplyStaging();
  }

  for (auto kf: GetAllKeyFrames()) {
    kf->GetCovisibilityGraph().Update();
  }

}

Atlas *Map::GetAtlas() const {
  return atlas_;
}

}
}
