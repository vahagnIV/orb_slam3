//
// Created by vahagn on 06/08/2021.
//

#include "base_feature_handler.h"

namespace orb_slam3 {
namespace features {
namespace handlers{

std::ostream & operator<<(std::ostream & stream, const BaseFeatureHandler * handler) {
  size_t size = handler->GetFeatures().Size();
  WRITE_TO_STREAM(size, stream);
  stream.write((char *)handler->GetFeatures().descriptors.data(), 32 * size);
  for(const auto & kp: handler->GetFeatures().keypoints){
    WRITE_TO_STREAM(kp.level, stream);
    WRITE_TO_STREAM(kp.size, stream);
    WRITE_TO_STREAM(kp.angle, stream);
    stream.write((char *)kp.pt.data(), kp.pt.size() * sizeof(decltype(kp.pt)::Scalar));
  }

  for(const auto & ukp: handler->GetFeatures().undistorted_keypoints)
    stream.write((char *)ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));

  for(const auto & ukp: handler->GetFeatures().undistorted_and_unprojected_keypoints)
    stream.write((char *)ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));

  return stream;
}

}
}
}