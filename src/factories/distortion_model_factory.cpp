//
// Created by vahagn on 29.09.21.
//

#include "distortion_model_factory.h"
#include <camera/distortions/barrel5.h>
#include <camera/distortions/barrel8.h>
#include <camera/distortions/fish_eye.h>

namespace orb_slam3 {
namespace factories {

camera::IDistortionModel *DistortionModelFactory::Create(camera::DistortionModelType type) {
  switch (type) {
    case camera::DistortionModelType::FISHEYE:
      return new camera::FishEye();
    case camera::DistortionModelType::BARREL5:
      return new camera::Barrel5();
    case camera::DistortionModelType::BARREL8:
      return new camera::Barrel8();
    default:
      return nullptr;
  }
  return nullptr;
}

}
}