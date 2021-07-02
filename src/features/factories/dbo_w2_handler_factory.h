//
// Created by vahagn on 01/07/2021.
//

#ifndef ORB_SLAM3_SRC_FEATURES_FACTORIES_D_BO_W_2_HANDLER_FACTORY_H_
#define ORB_SLAM3_SRC_FEATURES_FACTORIES_D_BO_W_2_HANDLER_FACTORY_H_

#include "handler_factory.h"
#include <features/bow_vocabulary.h>

namespace orb_slam3 {
namespace features {
namespace factories {

class DBoW2HandlerFactory : public HandlerFactory {
 public:
  DBoW2HandlerFactory(const BowVocabulary * vocabulary);
 public:
  std::shared_ptr<const handlers::BaseFeatureHandler> CreateFeatureHandler(Features & features,
                                                                      const features::IFeatureExtractor * feature_extractor) const override;
  frame::IKeyFrameDatabase * CreateKeyFrameDatabase() const override;
 private:
  const BowVocabulary * vocabulary_;

};

}
}
}

#endif //ORB_SLAM3_SRC_FEATURES_FACTORIES_D_BO_W_2_HANDLER_FACTORY_H_
