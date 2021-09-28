//
// Created by vahagn on 28.09.21.
//

#include "serializer.h"
#include <frame/key_frame.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace serialization {

void Serializer::Serialize(const map::Atlas *atlas, std::ostream &stream) {
  size_t map_count = atlas->GetMapCount();
  WRITE_TO_STREAM(map_count, stream);
  for (const auto map: atlas->GetMaps()) {
    Serialize(map, stream);
  }
}

void Serializer::Serialize(const map::Map *map, std::ostream &stream) {
  size_t kf_count = map->GetAllKeyFrames().size();
  WRITE_TO_STREAM(kf_count, stream);
  for (const auto kf: map->GetAllKeyFrames())
    Serialize(kf, stream);

  for (const auto mp: map->GetAllMapPoints())
    Serialize(mp, stream);

}

void Serializer::Serialize(const frame::KeyFrame *kf, std::ostream &stream) {
  size_t kf_id = kf->Id();
  WRITE_TO_STREAM(kf_id, stream);
  Serialize(kf->GetFilename(), stream);
  size_t map_id = reinterpret_cast<size_t>(kf->GetMap());
  assert(map_id > 0);
  WRITE_TO_STREAM(map_id, stream);
  stream << kf->GetPosition();
  kf->SerializeToStream(stream);
}

void Serializer::Serialize(const map::MapPoint *mp, std::ostream &stream) {
  size_t mp_id = reinterpret_cast<size_t>(mp);
  WRITE_TO_STREAM(mp_id, stream);
  stream.write((char *) mp->GetPosition().data(), 3 * sizeof(TPoint3D::Scalar));
  const map::MapPoint::MapType observations = mp->Observations();
  size_t observation_count = observations.size();
  WRITE_TO_STREAM(observation_count, stream);
  for (const auto obs: observations) {
    obs.
  }
}

void Serializer::Serialize(const std::string &string, std::ostream &stream) {
  size_t length = string.length();
  WRITE_TO_STREAM(length, stream);
  stream.write(string.data(), length);
}

void Serializer::Serialize(const frame::Observation &observation, std::ostream &stream) {
  size_t frame_id = observation.GetKeyFrame()->Id();
  size_t mem_address = reinterpret_cast<size_t>( observation.GetMapPoint());
  WRITE_TO_STREAM(mem_address, stream);
  WRITE_TO_STREAM(frame_id, stream);

  size_t feature_count = observation.GetFeatureIds().size();
  WRITE_TO_STREAM(feature_count, stream);

  for (size_t feature_id: observation.GetFeatureIds())
    WRITE_TO_STREAM(feature_id, stream);
}

}
}