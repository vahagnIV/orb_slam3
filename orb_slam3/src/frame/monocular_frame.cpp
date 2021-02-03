//
// Created by vahagn on 1/23/21.
//

#include <frame/monocular_frame.h>
#include <constants.h>
#include <features/second_nearest_neighbor_matcher.h>
#include <geometry/two_view_reconstructor.h>

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

bool MonocularFrame::InitializePositionFromPrevious() {
  if (previous_frame_->Type() != Type())
    return false;
  MonocularFrame * previous_frame = dynamic_cast<MonocularFrame *>(previous_frame_.get());
  features::SecondNearestNeighborMatcher matcher(200,
                                                 0.9,
                                                 false,
                                                 camera_->ImageBoundMinY(),
                                                 camera_->ImageBoundMinY(),
                                                 camera_->GridElementWidthInv(),
                                                 camera_->GridElementHeightInv());
  std::vector<int> matched_features;
  int number_of_good_matches = matcher.Match(features_, previous_frame->features_, matched_features);
  if (number_of_good_matches < 100)
    return false;

  geometry::TwoViewReconstructor reconstructor(camera_, camera_, 5);
  std::vector<TPoint3D> points;
  std::vector<bool> outliers;
  reconstructor.Reconstruct(features_.keypoints,
                            previous_frame->features_.keypoints,
                            matched_features,
                            pose_,
                            points,
                            outliers,
                            number_of_good_matches);

  //camera_->

  return false;
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

}
}  // namespace orb_slam3