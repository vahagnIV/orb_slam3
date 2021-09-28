//
// Created by vahagn on 27/09/2021.
//

#ifndef ORB_SLAM3_SRC_SERIALIZATION_SERIALIZATION_CONTEXT_H_
#define ORB_SLAM3_SRC_SERIALIZATION_SERIALIZATION_CONTEXT_H_
#include <unordered_map>

#include <map/atlas.h>
#include <typedefs.h>

namespace orb_slam3 {
namespace frame {
class Observation;
}
namespace serialization {

struct SerializationContext {
  std::unordered_map<const map::Map *, size_t> map_id;
};

class Serializer {
 public:
  static void Serialize(const map::Atlas *atlas, std::ostream &stream);
 private:
  static void Serialize(const map::Map *map, std::ostream &stream);
  static void Serialize(const frame::KeyFrame *kf, std::ostream &stream);
  static void Serialize(const map::MapPoint *mp, std::ostream &stream);
  static void Serialize(const frame::Observation &observation, std::ostream &stream);

 private:
  static void Serialize(const std::string &string, std::ostream &stream);

};

}
}

#endif //ORB_SLAM3_SRC_SERIALIZATION_SERIALIZATION_CONTEXT_H_
