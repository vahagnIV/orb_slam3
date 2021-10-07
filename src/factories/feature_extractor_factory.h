//
// Created by vahagn on 01.10.21.
//

#ifndef ORB_SLAM3_SRC_FACTORIES_FEATURE_EXTRACTOR_FACTORY_H_
#define ORB_SLAM3_SRC_FACTORIES_FEATURE_EXTRACTOR_FACTORY_H_
#include <features/ifeature_extractor.h>

namespace orb_slam3 {
namespace serialization {
class SerializationContext;
}
namespace factories {

class FeatureExtractorFactory {
 public:
  static features::IFeatureExtractor *Create(features::FeatureExtractorType type,
                                             std::istream &istream,
                                             serialization::SerializationContext &context);

};

}
}
#endif //ORB_SLAM3_SRC_FACTORIES_FEATURE_EXTRACTOR_FACTORY_H_
