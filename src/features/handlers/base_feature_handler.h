//
// Created by vahagn on 01/07/2021.
//

#ifndef ORB_SLAM3_SRC_FRAME_I_FEATURE_DRIVER_H_
#define ORB_SLAM3_SRC_FRAME_I_FEATURE_DRIVER_H_

#include <features/features.h>
#include <features/ifeature_extractor.h>
#include "handler_type.h"

namespace orb_slam3 {
namespace features {

typedef std::unordered_map<std::size_t, std::size_t> FastMatches;

enum MatchingSeverity {
  WEAK,
  MIDDLE,
  STRONG
};

namespace handlers {
class BaseFeatureHandler {
 public:
  virtual HandlerType Type() const = 0;
  BaseFeatureHandler(Features && features, const IFeatureExtractor * feature_extractor);
  BaseFeatureHandler(const features::IFeatureExtractor * feature_extractor);
  virtual ~BaseFeatureHandler() = default;
 public:
  void Serialize(std::ostream & ostream) const;
  void Deserialize(std::istream & istream, serialization::SerializationContext & context);

 public:
  const Features & GetFeatures() const { return features_; }
  const IFeatureExtractor * GetFeatureExtractor() const { return feature_extractor_; }

 public:
  virtual void FastMatch(const std::shared_ptr<const BaseFeatureHandler> & other,
                         FastMatches & out_matches,
                         MatchingSeverity severity,
                         bool check_orientation) const = 0;


 private:
  Features features_;
  const IFeatureExtractor * feature_extractor_;

};

}
}
}
#endif //ORB_SLAM3_SRC_FRAME_I_FEATURE_DRIVER_H_
