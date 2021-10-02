//
// Created by vahagn on 06/08/2021.
//

#include "base_feature_handler.h"
#include <serialization/serialization_context.h>

namespace orb_slam3 {
namespace features {
namespace handlers {

BaseFeatureHandler::BaseFeatureHandler(Features && features, const IFeatureExtractor * feature_extractor)
    : features_(features),
      feature_extractor_(feature_extractor) {}

BaseFeatureHandler::BaseFeatureHandler(istream &istream, serialization::SerializationContext &context)
    : features_(istream) {
  size_t feature_extractor_id;
  READ_FROM_STREAM(feature_extractor_id, istream);
  feature_extractor_ = context.fe_id[feature_extractor_id];

  features_.AssignFeaturesToGrid();
}

void BaseFeatureHandler::Serialize(std::ostream &stream) const {
  stream << features_;
  size_t feature_extractor_id = reinterpret_cast<size_t>(feature_extractor_);
  WRITE_TO_STREAM(feature_extractor_id, stream);
}

}
}
}