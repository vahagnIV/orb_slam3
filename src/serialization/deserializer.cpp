//
// Created by vahagn on 28.09.21.
//

#include "deserializer.h"

namespace orb_slam3 {
namespace serialization {

map::Atlas *Deserializer::Deserialize(std::istream &istream, SerializationContext &context) {
  auto atlas = new map::Atlas();
  size_t map_count;
  READ_FROM_STREAM(map_count, istream);
  for (size_t i = 0; i < map_count; ++i) {
    atlas->CreateNewMap() ;
    auto current_map = atlas->GetCurrentMap();
  }
  return nullptr;
}

}

}