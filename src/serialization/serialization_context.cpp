//
// Created by vahagn on 27/09/2021.
//

#include "serialization_context.h"
#include <frame/key_frame.h>

namespace orb_slam3 {
namespace serialization {

void Serializer::Serialize(const map::Atlas * atlas, std::ostream & stream, SerializationContext & context) {
  size_t map_count = atlas->GetMapCount();
  WRITE_TO_STREAM(map_count, stream);
  for (const auto map: atlas->GetMaps()) {
    Serialize(map, stream, context);
  }
}

void Serializer::Serialize(const map::Map * map, std::ostream & stream, SerializationContext & context) {
  context.map_id[map] = reinterpret_cast<size_t>(map);
  size_t kf_count = map->GetAllKeyFrames().size();
  WRITE_TO_STREAM(kf_count, stream);
  for (const auto kf: map->GetAllKeyFrames())
    Serialize(kf, stream, context);

  for (const auto mp: map->GetAllMapPoints())
    Serialize(mp, stream, context);

}

void Serializer::Serialize(const frame::KeyFrame * kf, std::ostream & stream, SerializationContext & context) {
  Serialize(kf->GetFilename(), stream);
  size_t map_id = context.map_id[kf->GetMap()];
  assert(map_id > 0);
  WRITE_TO_STREAM(map_id, stream);
  stream << kf->GetPosition();
  kf->SerializeToStream(stream);
}

void Serializer::Serialize(const map::MapPoint * mp, std::ostream & stream, SerializationContext & context) {

}

void Serializer::Serialize(const std::string & string, std::ostream & stream) {
  size_t length = string.length();
  WRITE_TO_STREAM(length, stream);
  stream.write(string.data(), length);
}

}
}
