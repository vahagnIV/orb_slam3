//
// Created by vahagn on 01/07/2021.
//

#ifndef ORB_SLAM3_SRC_FEATURES_FACTORIES_HANDLER_FACTORY_H_
#define ORB_SLAM3_SRC_FEATURES_FACTORIES_HANDLER_FACTORY_H_
#include <features/handlers/base_feature_handler.h>
#include <frame/database/ikey_frame_database.h>

namespace orb_slam3 {
namespace features {

class HandlerFactory {
 public:
  virtual std::shared_ptr<const handlers::BaseFeatureHandler> CreateFeatureHandler(Features & features,
                                                                              const features::IFeatureExtractor * feature_extractor) const = 0;
  virtual frame::IKeyFrameDatabase * CreateKeyFrameDatabase() const = 0;

};

}
}

#endif //ORB_SLAM3_SRC_FEATURES_FACTORIES_HANDLER_FACTORY_H_
