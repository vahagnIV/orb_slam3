//
// Created by vahagn on 29/09/2021.
//

#include "camera_factory.h"
#include "camera/monocular_camera.h"

namespace orb_slam3 {
namespace factories {

camera::ICamera * CameraFactory::CreateCamera(camera::CameraType type,
                                              std::istream &istream,
                                              serialization::SerializationContext &context) {
  switch (type) {
    case camera::CameraType::MONOCULAR:
      return new camera::MonocularCamera(istream, context);
    default:
      return nullptr;
  }
  return nullptr;
}

}
}