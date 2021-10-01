//
// Created by vahagn on 29/09/2021.
//

#ifndef ORB_SLAM3_SRC_FACTORIES_CAMERA_FACTORY_H_
#define ORB_SLAM3_SRC_FACTORIES_CAMERA_FACTORY_H_

#include "camera/icamera.h"

namespace orb_slam3 {
namespace serialization {
class SerializationContext;
}
namespace factories {

class CameraFactory {
 public:
  static camera::ICamera *CreateCamera(camera::CameraType type,
                                       std::istream &istream,
                                       serialization::SerializationContext &context);
};

}
}

#endif //ORB_SLAM3_SRC_FACTORIES_CAMERA_FACTORY_H_
