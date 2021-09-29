//
// Created by vahagn on 06/08/2021.
//

#include "base_feature_handler.h"

namespace orb_slam3 {
namespace features {
namespace handlers {

BaseFeatureHandler::BaseFeatureHandler(Features && features, const IFeatureExtractor * feature_extractor)
    : features_(features),
      feature_extractor_(feature_extractor) {}

BaseFeatureHandler::BaseFeatureHandler(const features::IFeatureExtractor * feature_extractor) : feature_extractor_(
    feature_extractor) {

}

void BaseFeatureHandler::Serialize(std::ostream & stream) const {
  size_t size = GetFeatures().Size();
  WRITE_TO_STREAM(size, stream);
  stream.write((char *) GetFeatures().descriptors.data(), 32 * size);
  for (const auto & kp: GetFeatures().keypoints) {
    WRITE_TO_STREAM(kp.level, stream);
    WRITE_TO_STREAM(kp.size, stream);
    WRITE_TO_STREAM(kp.angle, stream);
    stream.write((char *) kp.pt.data(), kp.pt.size() * sizeof(decltype(kp.pt)::Scalar));
  }

  for (const auto & ukp: GetFeatures().undistorted_keypoints)
    stream.write((char *) ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));

  for (const auto & ukp: GetFeatures().undistorted_and_unprojected_keypoints)
    stream.write((char *) ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));
}

void BaseFeatureHandler::Deserialize(std::istream & istream, serialization::SerializationContext & context) {
  size_t size = GetFeatures().Size();
  READ_FROM_STREAM(size, istream);
  istream.read((char *) GetFeatures().descriptors.data(), 32 * size);
  for (const auto & kp: GetFeatures().keypoints) {
    READ_FROM_STREAM(kp.level,istream);
    READ_FROM_STREAM(kp.size, istream);
    READ_FROM_STREAM(kp.angle,istream);
    istream.read((char *) kp.pt.data(), kp.pt.size() * sizeof(decltype(kp.pt)::Scalar));
  }

  for (const auto & ukp: GetFeatures().undistorted_keypoints)
    istream.read((char *) ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));

  for (const auto & ukp: GetFeatures().undistorted_and_unprojected_keypoints)
    istream.read((char *) ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));
}

}
}
}