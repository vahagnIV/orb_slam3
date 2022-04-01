//
// Created by vahagn on 29/09/2021.
//

#include <boost/filesystem.hpp>

#include "feature_handler_factory.h"
#include "features/handlers/DBoW2/dbo_w2_handler.h"
#include <serialization/serialization_context.h>
#include <frame/database/DBoW2/dbo_w2_database.h>
#include <src/features/bow/dbo_w2_vocabulary.h>
#include <camera/monocular_camera.h>

namespace orb_slam3 {
namespace factories {

FeatureHandlerFactory::FeatureHandlerFactory(features::handlers::HandlerType type, map::Atlas * atlas)
    : handler_type_(type), atlas_(atlas) {

}

FeatureHandlerFactory::~FeatureHandlerFactory() {
}

std::shared_ptr<features::handlers::BaseFeatureHandler> FeatureHandlerFactory::Create(features::handlers::HandlerType type,
                                                                                      std::istream & istream,
                                                                                      serialization::SerializationContext & context) {
  switch (type) {
    case features::handlers::HandlerType::DBoW2: {
      auto result = std::make_shared<features::handlers::DBoW2Handler>(istream,
                                                                       context,
                                                                       features::utils::DBoW2Vocabulary::Instance().GetVocabulary());
      result->Precompute();
      return result;
    }
    default:
      return nullptr;
  }
}
std::shared_ptr<features::handlers::BaseFeatureHandler> FeatureHandlerFactory::Create(features::handlers::HandlerType type,
                                                                                      const TImageGray8U & image,
                                                                                      const camera::ICamera * camera,
                                                                                      const features::IFeatureExtractor * feature_extractor,
                                                                                      size_t feature_count) {
  features::Features features(image.cols(), image.rows());

  feature_extractor->Extract(image, features, feature_count);


  if (camera->Type() == camera::CameraType::MONOCULAR) {
    auto mono_cam = dynamic_cast<const camera::MonocularCamera *>(camera);
    features.undistorted_keypoints.reserve(features.Size());
    features.undistorted_and_unprojected_keypoints.reserve(features.Size());

    size_t feature_size = features.Size();
    for (size_t i = 0; i < feature_size;) {
      TPoint2D undistorted_kp;
      if (mono_cam->UndistortPoint(features.keypoints[i].pt, undistorted_kp)) {
        TPoint3D undistorted_unprojected_kp;
        mono_cam->UnprojectAndUndistort(features.keypoints[i].pt,
                                        undistorted_unprojected_kp);
        features.undistorted_keypoints.emplace_back(undistorted_kp);
        features.undistorted_and_unprojected_keypoints.emplace_back(undistorted_unprojected_kp);
        ++i;
      } else {
        --feature_size;
        std::swap(features.keypoints[i], features.keypoints[feature_size - i ]);
        features.descriptors.row(i).swap( features.descriptors.row(feature_size - i ));
      }
    }
    features.keypoints.resize(feature_size);
    features.descriptors.conservativeResize(feature_size, features.descriptors.cols());
  }
  features.AssignFeaturesToGrid();
  switch (type) {
    case features::handlers::HandlerType::DBoW2: {

      auto result = std::make_shared<features::handlers::DBoW2Handler>(std::move(features),
                                                                       feature_extractor,
                                                                       features::utils::DBoW2Vocabulary::Instance().GetVocabulary());
      result->Precompute();
      return result;
    }
    default:
      return nullptr;
  }

}

}
}