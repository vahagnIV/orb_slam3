//
// Created by vahagn on 1/23/21.
//

// == orb-slam3 ===
#include <frame/monocular_frame.h>
#include <constants.h>
#include <features/second_nearest_neighbor_matcher.h>
#include <geometry/two_view_reconstructor.h>

namespace orb_slam3 {
namespace frame {

MonocularFrame::MonocularFrame(const TImageGray8U & image, TimePoint timestamp,
                               const std::shared_ptr<features::IFeatureExtractor> & feature_extractor,
                               const std::shared_ptr<camera::MonocularCamera> & camera) :
    FrameBase(timestamp),
    features_(camera->Width(), camera->Height()),
    camera_(camera) {
  feature_extractor->Extract(image, features_);
  features_.UndistortKeyPoints(camera_);
  features_.AssignFeaturesToGrid();
  map_points_.resize(features_.undistorted_keypoints.size());
  std::fill(map_points_.begin(), map_points_.end(), nullptr);
}

bool MonocularFrame::IsValid() const {
  return FeatureCount() > constants::MINIMAL_FEATURE_COUNT_PER_FRAME_MONOCULAR;
}

size_t MonocularFrame::FeatureCount() const noexcept {
  return features_.keypoints.size();
}

bool MonocularFrame::Link(const std::shared_ptr<FrameBase> & other) {
  if (other->Type() != Type())
    return false;
  MonocularFrame * other_frame = dynamic_cast<MonocularFrame *>(other.get());
  features::SecondNearestNeighborMatcher matcher(100,
                                                 0.9,
                                                 false);
  matcher.Match(features_, other_frame->features_, frame_link_.matches);
  if (frame_link_.matches.size() < 100)
    return false;

  geometry::TwoViewReconstructor reconstructor(5, camera_->FxInv());
  std::vector<TPoint3D> points;
  std::vector<bool> outliers;
  if (reconstructor.Reconstruct(features_.undistorted_keypoints,
                                other_frame->features_.undistorted_keypoints,
                                frame_link_.matches,
                                pose_,
                                points,
                                frame_link_.inliers)) {
    // TODO: pass to asolute R,T
    frame_link_.other = other;

    for (size_t i = 0; i < frame_link_.matches.size(); ++i) {
      if (!frame_link_.inliers[i])
        continue;
      const features::Match & match = frame_link_.matches[i];

      if (other->MapPoint(match.from_idx)) {
        // TODO: do the contistency check
      } else {

        auto map_point = std::make_shared<map::MapPoint>(points[i]);
        map_points_[match.to_idx] = map_point;
        other->MapPoint(match.from_idx) = map_points_[match.to_idx];
      }
    }

    std::cout << "Frame " << Id() << " " << pose_.R << std::endl << pose_.T << std::endl;
    return true;
  }

  return false;
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

}
}  // namespace orb_slam3