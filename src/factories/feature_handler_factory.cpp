//
// Created by vahagn on 29/09/2021.
//

#include <boost/filesystem.hpp>

#include "feature_handler_factory.h"
#include "features/handlers/DBoW2/dbo_w2_handler.h"
#include <serialization/serialization_context.h>
#include <frame/database/DBoW2/dbo_w2_database.h>

namespace orb_slam3 {
namespace factories {

FeatureHandlerFactory::FeatureHandlerFactory() : bow_vocabulary_(nullptr) {

}

FeatureHandlerFactory &FeatureHandlerFactory::Instance() {
  static FeatureHandlerFactory instance;
  return instance;
}

FeatureHandlerFactory::~FeatureHandlerFactory() {
  delete bow_vocabulary_;
}

void FeatureHandlerFactory::LoadBowVocabulary() {
  if (bow_vocabulary_)
    return;

  const char *val = std::getenv(constants::BOW_VOCABULARY_FILE_PATH.c_str());
  if (nullptr == val) {
    std::stringstream ss;
    ss << "Could not find the environment variable " << constants::BOW_VOCABULARY_FILE_PATH;
    throw std::runtime_error(ss.str());
  }
  std::string bow_vocabulary_file_path(val);
  if (!boost::filesystem::exists(bow_vocabulary_file_path)) {
    std::stringstream ss;
    ss << "Error while loading bow vocabulary " << bow_vocabulary_file_path << ". No such file or directory.";
    throw std::runtime_error(ss.str());
  }
  bow_vocabulary_ = new orb_slam3::features::BowVocabulary;
  bow_vocabulary_->loadFromTextFile(bow_vocabulary_file_path);

}

std::shared_ptr<features::handlers::BaseFeatureHandler> FeatureHandlerFactory::Create(features::handlers::HandlerType type,
                                                                                      std::istream &istream,
                                                                                      serialization::SerializationContext &context) {
  switch (type) {
    case features::handlers::HandlerType::DBoW2: {
      LoadBowVocabulary();
      auto result = std::make_shared<features::handlers::DBoW2Handler>(istream, context, bow_vocabulary_);
      result->Precompute();
      return result;
    }
    default:
      return nullptr;
  }
}
std::shared_ptr<features::handlers::BaseFeatureHandler> FeatureHandlerFactory::Create(features::handlers::HandlerType type,
                                                                                      const TImageGray8U & image,
                                                                                      camera::ICamera * camera,
                                                                                      const features::IFeatureExtractor *feature_extractor) {
  switch (type) {
    case features::handlers::HandlerType::DBoW2: {
      LoadBowVocabulary();
      features::Features features(image.cols(), image.rows());

      feature_extractor->Extract(image, features, 0);
      features.AssignFeaturesToGrid();

      if(camera->Type() == camera::CameraType::MONOCULAR) {
        auto mono_cam = dynamic_cast<camera::MonocularCamera *>(camera);
        features.undistorted_keypoints.resize(features.Size());
        features.undistorted_and_unprojected_keypoints.resize(features.Size());
        for (size_t i = 0; i < features.Size(); ++i) {
          mono_cam->UndistortPoint(features.keypoints[i].pt, features.undistorted_keypoints[i]);
          mono_cam->UnprojectAndUndistort(features.keypoints[i].pt, features.undistorted_and_unprojected_keypoints[i]);
        }
      }
      auto result = std::make_shared<features::handlers::DBoW2Handler>(std::move(features),
                                                                       feature_extractor,
                                                                       bow_vocabulary_);
      result->Precompute();
      return result;
    }
    default:
      return nullptr;
  }


}
frame::IKeyFrameDatabase *FeatureHandlerFactory::CreateKeyFrameDatabase(features::handlers::HandlerType type) {
  switch (type) {
    case features::handlers::HandlerType::DBoW2: {
      LoadBowVocabulary();
      return new frame::DBoW2Database(bow_vocabulary_);
    }
    default:
      return nullptr;
  }
}

}
}