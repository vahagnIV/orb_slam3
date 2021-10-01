//
// Created by vahagn on 28.09.21.
//

#include <frame/frame_type.h>
#include "deserializer.h"

namespace orb_slam3 {
namespace serialization {

map::Atlas * Deserializer::Deserialize(std::istream & istream, SerializationContext & context) {
//  auto atlas = new map::Atlas();
//  size_t map_count;
//  READ_FROM_STREAM(map_count, istream);
//  for (size_t i = 0; i < map_count; ++i) {
//    atlas->SetCurrentMap(DeserializeMap(istream, context));
//  }
//  return atlas;
}

map::Map * Deserializer::DeserializeMap(std::istream & istream, SerializationContext & context) {
  auto map = new map::Map();
  size_t map_id;
  READ_FROM_STREAM(map_id, istream);
  context.map_id[map_id] = map;

  size_t kf_count;
  READ_FROM_STREAM(kf_count, istream);

  for (size_t kf_num = 0; kf_num < kf_count; ++kf_num) {
    auto kf = DeserializeKeyFrame(istream, context);
    map->AddKeyFrame(kf);
  }

  return nullptr;
}

frame::KeyFrame * Deserializer::DeserializeKeyFrame(std::istream & istream, SerializationContext & context) {
  frame::FrameType type;
  READ_FROM_STREAM(type, istream);
  if (frame::FrameType::MONOCULAR == type) {
//    fr
  }
  return nullptr;
}

}

}