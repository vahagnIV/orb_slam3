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
#warning remember feature extractor id;
  Eigen::Index features_width = GetFeatures().descriptors.cols();
  Eigen::Index features_height = GetFeatures().descriptors.rows();
  WRITE_TO_STREAM(features_width, stream);
  WRITE_TO_STREAM(features_height, stream);
  stream.write((char *) GetFeatures().descriptors.data(),
               features_width * features_height * sizeof(decltype(GetFeatures().descriptors)::Scalar));
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
#warning read feature extractor id;
  Eigen::Index features_width;
  Eigen::Index features_height;
  READ_FROM_STREAM(features_width, istream);
  READ_FROM_STREAM(features_height, istream);
  features_.descriptors.resize(features_height, features_width);

  istream.read((char *) features_.descriptors.data(),
               features_width * features_height * sizeof(decltype(features_.descriptors)::Scalar));
  for (Eigen::Index i = 0; i < features_height; ++i) {
    features::KeyPoint kp;
    READ_FROM_STREAM(kp.level, istream);
    READ_FROM_STREAM(kp.size, istream);
    READ_FROM_STREAM(kp.angle, istream);
    istream.read((char *) kp.pt.data(), kp.pt.size() * sizeof(decltype(kp.pt)::Scalar));
    features_.keypoints.push_back(kp);
  }

  for (Eigen::Index i = 0; i < features_height; ++i) {
    TPoint2D ukp;
    istream.read((char *) ukp.data(), ukp.size() * sizeof(std::remove_reference<decltype(ukp)>::type::Scalar));
    features_.undistorted_keypoints.emplace_back(ukp);
  }

  for (Eigen::Index i = 0; i < features_height; ++i) {
    TPoint3D uukp;
    istream.read((char *) uukp.data(), uukp.size() * sizeof(std::remove_reference<decltype(uukp)>::type::Scalar));
    features_.undistorted_and_unprojected_keypoints.emplace_back(uukp);
  }
}

}
}
}