//
// Created by vahagn on 1/23/21.
//

#include <frame/monocular_frame.h>
#include <constants.h>

namespace orb_slam3 {
namespace frame {

MonocularFrame::MonocularFrame(const TImageGray8U & image, TimePoint timestamp,
                               const std::shared_ptr<features::IFeatureExtractor> &
                               feature_extractor,
                               const std::shared_ptr<camera::MonocularCamera> & camera) :
    FrameBase(timestamp,
              feature_extractor),
    camera_(camera) {
  std::vector<features::KeyPoint> key_points;
  feature_extractor_->Extract(image, features_);
  camera_->UndistortKeyPoints(features_.keypoints, features_.undistorted_keypoints);
  features_.AssignFeaturesToGrid(camera_->ImageBoundMinX(),
                                 camera_->ImageBoundMinY(),
                                 camera_->GridElementWidthInv(),
                                 camera_->GridElementHeightInv());
}

bool MonocularFrame::IsValid() const {
  return FeatureCount() > constants::MINIMAL_FEATURE_COUNT_PER_FRAME_MONOCULAR;
}

size_t MonocularFrame::FeatureCount() const noexcept {
  return features_.keypoints.size();
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

}
}  // namespace orb_slam3