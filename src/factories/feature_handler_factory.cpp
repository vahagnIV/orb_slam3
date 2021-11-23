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
  switch (type) {
    case features::handlers::HandlerType::DBoW2: {

      features::Features features(image.cols(), image.rows());

      feature_extractor->Extract(image, features, feature_count);
      features.AssignFeaturesToGrid();

      if (camera->Type() == camera::CameraType::MONOCULAR) {
        auto mono_cam = dynamic_cast<const camera::MonocularCamera *>(camera);
        features.undistorted_keypoints.resize(features.Size());
        features.undistorted_and_unprojected_keypoints.resize(features.Size());
        for (size_t i = 0; i < features.Size(); ++i) {
          mono_cam->UndistortPoint(features.keypoints[i].pt, features.undistorted_keypoints[i]);
          mono_cam->UnprojectAndUndistort(features.keypoints[i].pt, features.undistorted_and_unprojected_keypoints[i]);
        }
      }
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