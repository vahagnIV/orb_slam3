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
              feature_extractor), features_(camera->Width(), camera->Height()),
    camera_(camera) {
  feature_extractor_->Extract(image, features_);
  features_.UndistortKeyPoints(camera_);
  features_.AssignFeaturesToGrid();
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
                                                 false);
  std::vector<int> matched_features;
  int number_of_good_matches = matcher.Match(features_, previous_frame->features_, matched_features);
  if (number_of_good_matches < 50)
    return false;

  geometry::TwoViewReconstructor reconstructor(camera_, camera_, 5, 1. / 190.);
  std::vector<TPoint3D> points;
  std::vector<bool> outliers;
  if (reconstructor.Reconstruct(features_.undistorted_keypoints,
                                previous_frame->features_.undistorted_keypoints,
                                matched_features,
                                pose_,
                                points,
                                outliers,
                                number_of_good_matches)) {
    std::cout << pose_.R << std::endl << pose_.T << std::endl;
    return true;
  }

  return false;
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

}
}  // namespace orb_slam3