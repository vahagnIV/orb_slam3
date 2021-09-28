//
// Created by vahagn on 27/09/2021.
//

#ifndef ORB_SLAM3_SRC_SERIALIZATION_SERIALIZATION_CONTEXT_H_
#define ORB_SLAM3_SRC_SERIALIZATION_SERIALIZATION_CONTEXT_H_
#include <unordered_map>

#include <map/atlas.h>
#include <typedefs.h>

namespace orb_slam3 {

namespace serialization {

struct SerializationContext {
  std::unordered_map<size_t, map::Map *> map_id;
};



}
}

#endif //ORB_SLAM3_SRC_SERIALIZATION_SERIALIZATION_CONTEXT_H_
