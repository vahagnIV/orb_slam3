//
// Created by vahagn on 29/09/2021.
//

#ifndef ORB_SLAM3_SRC_FACTORIES_FEATURE_HANDLER_FACTORY_H_
#define ORB_SLAM3_SRC_FACTORIES_FEATURE_HANDLER_FACTORY_H_

#include <features/handlers/base_feature_handler.h>

namespace orb_slam3 {

namespace serialization{
class SerializationContext;
}

namespace factories {

class FeatureHandlerFactory {
 public:
  static std::shared_ptr<features::handlers::BaseFeatureHandler> Create(features::handlers::HandlerType type,
                                                                        std::istream &istream,
                                                                        serialization::SerializationContext &context);
};

}
}
#endif //ORB_SLAM3_SRC_FACTORIES_FEATURE_HANDLER_FACTORY_H_
