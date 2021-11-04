//
// Created by vahagn on 29/09/2021.
//

#ifndef ORB_SLAM3_SRC_FACTORIES_FEATURE_HANDLER_FACTORY_H_
#define ORB_SLAM3_SRC_FACTORIES_FEATURE_HANDLER_FACTORY_H_

#include <features/handlers/base_feature_handler.h>
#include <src/features/bow/bow_vocabulary.h>
#include <frame/database/ikey_frame_database.h>

namespace orb_slam3 {

namespace map {
class Atlas;
}

namespace serialization {
class SerializationContext;
}

namespace factories {

class FeatureHandlerFactory {
 public:
  FeatureHandlerFactory(features::handlers::HandlerType type, map::Atlas * atlas);

  static std::shared_ptr<features::handlers::BaseFeatureHandler> Create(features::handlers::HandlerType type,
                                                                        std::istream & istream,
                                                                        serialization::SerializationContext & context);

  static std::shared_ptr<features::handlers::BaseFeatureHandler> Create(features::handlers::HandlerType type,
                                                                        const TImageGray8U & image,
                                                                        const camera::ICamera * camera,
                                                                        const features::IFeatureExtractor * feature_extractor,
                                                                        size_t feature_count);



  ~FeatureHandlerFactory();

 private:
  features::handlers::HandlerType handler_type_;
  map::Atlas * atlas_;

};

}
}
#endif //ORB_SLAM3_SRC_FACTORIES_FEATURE_HANDLER_FACTORY_H_
