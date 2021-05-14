//
// Created by vahagn on 11/05/2021.
//

#include <g2o/core/sparse_optimizer.h>

#include "monocular_frame.h"
#include <logging.h>
#include <features/matching/second_nearest_neighbor_matcher.hpp>
#include <features/matching/iterators/area_to_iterator.h>
#include <features/matching/orientation_validator.h>
#include <geometry/two_view_reconstructor.h>
#include <optimization/bundle_adjustment.h>
#include "monocular_key_frame.h"

namespace orb_slam3 {
namespace frame {
namespace monocular {

MonocularFrame::MonocularFrame(const TImageGray8U & image,
                               TimePoint time_point,
                               const string & filename,
                               const features::IFeatureExtractor * feature_extractor,
                               const camera::MonocularCamera * camera,
                               const features::BowVocabulary * vocabulary,
                               const SensorConstants * sensor_constants) : Frame(time_point,
                                                                                 filename,
                                                                                 feature_extractor,
                                                                                 vocabulary,
                                                                                 sensor_constants),
                                                                           BaseMonocular(image.cols(),
                                                                                         image.rows(),
                                                                                         camera) {
  const_cast<features::IFeatureExtractor *>(feature_extractor)->Extract(image, features_);
  features_.UndistortKeyPoints(GetCamera());
  features_.AssignFeaturesToGrid();
  features_.SetVocabulary(vocabulary);

}

bool MonocularFrame::Link(Frame * other) {
  logging::RetrieveLogger()->info("Linking frame {} with {}", Id(), other->Id());
  if (other->Type() != Type()) {
    throw std::runtime_error("Linking frames of different types is not implemented yet.");
  }

  MonocularFrame * from_frame = dynamic_cast<MonocularFrame *>(other);
  std::unordered_map<std::size_t, std::size_t> matches;
  if (!ComputeMatchesForLinking(from_frame, matches))
    return false;

  geometry::TwoViewReconstructor reconstructor(200, GetCamera()->FxInv());
  std::unordered_map<size_t, TPoint3D> points;
  geometry::Pose pose;
  if (!reconstructor.Reconstruct(GetFeatures().undistorted_and_unprojected_keypoints,
                                 from_frame->GetFeatures().undistorted_and_unprojected_keypoints,
                                 matches,
                                 pose,
                                 points)) {

    logging::RetrieveLogger()->info("LINKING: Could not reconstruct points between frames {} and {}. Skipping...",
                                    Id(),
                                    other->Id());
    return false;
  }
//#ifndef NDEBUG
//  cv::imshow("linked matches:",
//             debug::DrawMatches(Filename(), other->Filename(), matches, features_, from_frame->features_));
//  cv::waitKey(1);
//#endif

  this->SetPosition(pose);
  std::unordered_set<map::MapPoint *> map_points;
  InitializeMapPointsFromMatches(matches, points, from_frame, map_points);
  std::cout << " Mps: " << map_points.size() << std::endl;
  return true;
}

bool MonocularFrame::EstimatePositionFromReferenceKeyframe(const KeyFrame * reference_keyframe) {
  return false;
}

bool MonocularFrame::EstimatePositionByProjectingMapPoints(const MapPointSet & map_points) {
  return false;
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

KeyFrame * MonocularFrame::CreateKeyFrame() {
  return new MonocularKeyFrame(this);
}

void MonocularFrame::ListMapPoints(BaseFrame::MapPointSet & out_map_points) const {
  BaseMonocular::ListMapPoints(out_map_points);
}

bool MonocularFrame::ComputeMatchesForLinking(MonocularFrame * from_frame,
                                              unordered_map<size_t, size_t> & out_matches) {
  features::matching::SNNMatcher<features::matching::iterators::AreaToIterator> matcher(0.9, 50);
  features::matching::iterators::AreaToIterator begin(0, &GetFeatures(), &from_frame->GetFeatures(), 100);
  features::matching::iterators::AreaToIterator
      end(GetFeatures().Size(), &GetFeatures(), &from_frame->GetFeatures(), 100);

  matcher.MatchWithIteratorV2(begin, end, feature_extractor_, out_matches);
  logging::RetrieveLogger()->debug("Orientation validator discarderd {} matches",
                                   features::matching::OrientationValidator
                                       (GetFeatures().keypoints, from_frame->GetFeatures().keypoints).Validate(
                                       out_matches));

  logging::RetrieveLogger()->debug("Link: SNN Matcher returned {} matches between frames {} and {}.",
                                   out_matches.size(),
                                   Id(),
                                   from_frame->Id());
  if (out_matches.size() < 100) {
    logging::RetrieveLogger()->debug("Not enough matches. Skipping");
    return false;
  }
  return true;
}

void MonocularFrame::InitializeMapPointsFromMatches(const std::unordered_map<std::size_t, std::size_t> & matches,
                                                    const std::unordered_map<size_t, TPoint3D> & points,
                                                    MonocularFrame * from_frame,
                                                    BaseFrame::MapPointSet & out_map_points) {
  for (const auto & point : points) {

    precision_t max_invariance_distance, min_invariance_distance;
    feature_extractor_->ComputeInvariantDistances(GetPosition().Transform(point.second),
                                                  GetFeatures().keypoints[point.first],
                                                  max_invariance_distance,
                                                  min_invariance_distance);
    auto map_point = new map::MapPoint(point.second, max_invariance_distance, min_invariance_distance, Id());
    AddMapPoint(map_point, point.first);
    from_frame->AddMapPoint(map_point, matches.find(point.first)->second);
    out_map_points.insert(map_point);
  }
}

bool MonocularFrame::IsValid() const {
  return GetFeatures().Size() > 0;
}

void MonocularFrame::ComputeBow() {
  BaseMonocular::ComputeBow();
}

}
}
}