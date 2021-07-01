//
// Created by vahagn on 01/07/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_I_FEATURE_DRIVER_H_
#define ORB_SLAM3_SRC_FRAME_I_FEATURE_DRIVER_H_

#include <features/features.h>
#include <features/ifeature_extractor.h>

namespace orb_slam3 {
namespace features {

typedef std::unordered_map<std::size_t, std::size_t> FastMatches;

namespace handlers {
class BaseFeatureHandler {
 public:
  BaseFeatureHandler(Features && features, const IFeatureExtractor * feature_extractor)
      : features_(features),
        feature_extractor_(feature_extractor) {}

 public:
  const Features & GetFeatures() const { return features_; }
  const IFeatureExtractor * GetFeatureExtractor() const { return feature_extractor_; }
 public:
  virtual void FastMatch(const BaseFeatureHandler & other, FastMatches & out_matches) = 0;

 private:
  const Features features_;
  const IFeatureExtractor * feature_extractor_;

};

}
}
}
#endif //ORB_SLAM3_SRC_FRAME_I_FEATURE_DRIVER_H_
