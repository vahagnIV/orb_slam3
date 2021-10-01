//
// Created by vahagn on 27/09/2021.
//

#ifndef ORB_SLAM3_SRC_SERIALIZATION_SERIALIZATION_CONTEXT_H_
#define ORB_SLAM3_SRC_SERIALIZATION_SERIALIZATION_CONTEXT_H_
#include <unordered_map>

#include <typedefs.h>
#include <camera/icamera.h>
#include <map/map.h>
#include <features/bow_vocabulary.h>
#include <features/ifeature_extractor.h>
#include <frame/sensor_constants.h>
#include <features/ifeature_extractor.h>

namespace orb_slam3 {
namespace serialization {

struct SerializationContext {
  std::unordered_map<size_t, camera::ICamera*> cam_id;
  std::unordered_map<size_t, map::Map *> map_id;
  std::unordered_map<size_t, frame::KeyFrame *> kf_id;
  std::unordered_map<size_t, map::MapPoint *> mp_id;
  std::unordered_map<size_t, frame::SensorConstants *> sc_id;
  std::unordered_map<size_t, features::IFeatureExtractor *> fe_id;
  features::BowVocabulary * vocabulary;
  features::IFeatureExtractor * feature_extractor;
};

}
}

#endif //ORB_SLAM3_SRC_SERIALIZATION_SERIALIZATION_CONTEXT_H_
