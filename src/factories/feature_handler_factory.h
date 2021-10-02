//
// Created by vahagn on 29/09/2021.
//

#ifndef ORB_SLAM3_SRC_FACTORIES_FEATURE_HANDLER_FACTORY_H_
#define ORB_SLAM3_SRC_FACTORIES_FEATURE_HANDLER_FACTORY_H_

#include <features/handlers/base_feature_handler.h>
#include <features/bow_vocabulary.h>
#include <frame/database/ikey_frame_database.h>

namespace orb_slam3 {

namespace serialization {
class SerializationContext;
}

namespace factories {

class FeatureHandlerFactory {
 public:
  static FeatureHandlerFactory &Instance();
  std::shared_ptr<features::handlers::BaseFeatureHandler> Create(features::handlers::HandlerType type,
                                                                 std::istream &istream,
                                                                 serialization::SerializationContext &context);

  std::shared_ptr<features::handlers::BaseFeatureHandler> Create(features::handlers::HandlerType type,
                                                                 const TImageGray8U &image,
                                                                 camera::ICamera *camera,
                                                                 const features::IFeatureExtractor *feature_extractor);
  frame::IKeyFrameDatabase *CreateKeyFrameDatabase(features::handlers::HandlerType type);
  ~FeatureHandlerFactory();
 private:
  FeatureHandlerFactory();
 private:
  void LoadBowVocabulary();
 private:

  features::BowVocabulary *bow_vocabulary_;

};

}
}
#endif //ORB_SLAM3_SRC_FACTORIES_FEATURE_HANDLER_FACTORY_H_
