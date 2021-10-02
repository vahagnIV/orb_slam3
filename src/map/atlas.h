//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_ATLAS_H_
#define ORB_SLAM3_INCLUDE_ATLAS_H_
// === stl ===
#include <memory>

// === orb-slam3 ===
#include "map.h"
#include <features/ifeature_extractor.h>

namespace orb_slam3 {
namespace serialization {
class SerializationContext;
}

namespace frame {
class IKeyFrameDatabase;
}

namespace map {

class Atlas {
 public:
  explicit Atlas(features::IFeatureExtractor *feature_extractor, frame::IKeyFrameDatabase *keyframe_database);
  Atlas(std::istream &istream, serialization::SerializationContext &context);
  ~Atlas();

  Map *GetCurrentMap();
  void CreateNewMap();
  size_t GetMapCount() const;
  const std::unordered_set<map::Map *> &GetMaps() const;
  const features::IFeatureExtractor *GetFeatureExtractor() const;
  frame::IKeyFrameDatabase * GetKeyframeDatabase() const;

  void SetCurrentMap(map::Map *map);
  void Serialize(std::ostream &ostream) const;
 private:
  Map *current_map_;
  std::unordered_set<map::Map *> maps_;
  features::IFeatureExtractor *feature_extractor_;
  frame::IKeyFrameDatabase *key_frame_database_;
};

}
}
#endif //ORB_SLAM3_INCLUDE_ATLAS_H_
