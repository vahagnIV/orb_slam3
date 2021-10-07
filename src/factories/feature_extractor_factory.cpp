//
// Created by vahagn on 01.10.21.
//

#include "feature_extractor_factory.h"
#include <features/orb_feature_extractor.h>

namespace orb_slam3 {
namespace factories {
features::IFeatureExtractor *FeatureExtractorFactory::Create(features::FeatureExtractorType type,
                                                             std::istream &istream,
                                                             serialization::SerializationContext &context) {
  switch (type) {
    case features::FeatureExtractorType::ORBFE:
      return new features::ORBFeatureExtractor(istream, context);
  }
  return nullptr;
}
}
}