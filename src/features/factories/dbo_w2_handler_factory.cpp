//
// Created by vahagn on 01/07/2021.
//

#include "dbo_w2_handler_factory.h"
#include <features/handlers/DBoW2/dbo_w2_handler.h>
#include <frame/database/DBoW2/dbo_w2_database.h>

#include <memory>

namespace orb_slam3 {
namespace features {
namespace factories {

DBoW2HandlerFactory::DBoW2HandlerFactory(const BowVocabulary *vocabulary)
    : vocabulary_(vocabulary) {

}

shared_ptr<const handlers::BaseFeatureHandler> DBoW2HandlerFactory::CreateFeatureHandler(Features & features,
                                                                                         const features::IFeatureExtractor * feature_extractor) const {
  auto result = std::make_shared<handlers::DBoW2Handler>(std::move(features),
                                                         feature_extractor,
                                                         vocabulary_);
  result->Precompute();
  return result;
}

frame::IKeyFrameDatabase * DBoW2HandlerFactory::CreateKeyFrameDatabase() const {
  return new frame::DBoW2Database(vocabulary_);
}

}
}
}