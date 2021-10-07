//
// Created by vahagn on 06/08/2021.
//

#include "base_feature_handler.h"
#include <serialization/serialization_context.h>
#include <map/atlas.h>

namespace orb_slam3 {
namespace features {
namespace handlers {

BaseFeatureHandler::BaseFeatureHandler(Features && features, const IFeatureExtractor * feature_extractor)
    : features_(features),
      feature_extractor_(feature_extractor) {}

BaseFeatureHandler::BaseFeatureHandler(std::istream &istream, serialization::SerializationContext &context)
    : features_(istream), feature_extractor_(context.atlas->GetFeatureExtractor()) {
  features_.AssignFeaturesToGrid();
}

void BaseFeatureHandler::Serialize(std::ostream &stream) const {
  stream << features_;
}

}
}
}