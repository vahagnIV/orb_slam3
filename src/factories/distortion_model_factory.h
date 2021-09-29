//
// Created by vahagn on 29.09.21.
//

#ifndef ORB_SLAM3_SRC_FACTORIES_DISTORTION_MODEL_FACTORY_H_
#define ORB_SLAM3_SRC_FACTORIES_DISTORTION_MODEL_FACTORY_H_

#include <camera/distortions/idistortion_model.h>

namespace orb_slam3 {
namespace factories {

class DistortionModelFactory {
 public:
  static camera::IDistortionModel *Create(camera::DistortionModelType type);
};

}
}
#endif //ORB_SLAM3_SRC_FACTORIES_DISTORTION_MODEL_FACTORY_H_
