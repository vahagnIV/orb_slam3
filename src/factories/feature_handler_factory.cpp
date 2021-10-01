//
// Created by vahagn on 29/09/2021.
//

#include "feature_handler_factory.h"
#include "features/handlers/DBoW2/dbo_w2_handler.h"
#include <serialization/serialization_context.h>

namespace orb_slam3 {
namespace factories {

std::shared_ptr<features::handlers::BaseFeatureHandler> FeatureHandlerFactory::Create(features::handlers::HandlerType type,
                                                                                      std::istream &istream,
                                                                                      serialization::SerializationContext &context) {
  switch (type) {
    case features::handlers::HandlerType::DBoW2: {
      auto result = std::make_shared<features::handlers::DBoW2Handler>(istream, context, context.vocabulary);
      result->Precompute();
      return result;
    }
    default:
      return nullptr;
  }
  return std::shared_ptr<features::handlers::BaseFeatureHandler>(nullptr);
}

}
}