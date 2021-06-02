//
// Created by vahagn on 11/05/2021.
//

#include "monocular_frame.h"
#include <logging.h>
#include <features/matching/second_nearest_neighbor_matcher.hpp>
#include <features/matching/iterators/area_to_iterator.h>
#include <features/matching/iterators/projection_search_iterator.h>
#include <features/matching/orientation_validator.h>
#include <features/ifeature_extractor.h>
#include <map/map_point.h>
#include <geometry/two_view_reconstructor.h>
#include <optimization/monocular_optimization.h>
#include <features/matching/iterators/bow_to_iterator.h>
#include "monocular_key_frame.h"

#include <debug/debug_utils.h>

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
                                                                           BaseMonocular(camera),
                                                                           reference_keyframe_(nullptr) {
  const_cast<features::IFeatureExtractor *>(feature_extractor)->Extract(image, features_);
  features_.UndistortKeyPoints();
  features_.AssignFeaturesToGrid();
  features_.SetVocabulary(vocabulary);
  features_.ComputeBow();
}

bool MonocularFrame::Link(Frame * other) {
  logging::RetrieveLogger()->info("Linking frame {} with {}", Id(), other->Id());
  if (other->Type() != Type()) {
    throw std::runtime_error("Linking frames of different types is not implemented yet.");
  }

  auto from_frame = dynamic_cast<MonocularFrame *>(other);
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
  for(auto it = matches.begin(); it!=matches.end();){
    if(points.find(it->first) == points.end()){
      it = matches.erase(it);
    }
    else
      ++it;
  }
  cv::imshow("Linking",
             debug::DrawMatches(GetFilename(), other->GetFilename(), matches, features_, from_frame->features_));
  cv::waitKey();

  this->SetPosition(pose);
  std::unordered_set<map::MapPoint *> map_points;
  InitializeMapPointsFromMatches(matches, points, from_frame, map_points);
  logging::RetrieveLogger()->debug("Linking found {} matches", map_points_.size());
  return true;
}

bool MonocularFrame::FindMapPointsFromReferenceKeyFrame(const KeyFrame * reference_keyframe) {
  logging::RetrieveLogger()->info("TWRKF: Tracking frame {} with reference keyframe {}",
                                  Id(),
                                  reference_keyframe->Id());
  if (reference_keyframe->Type() != Type()) {
    logging::RetrieveLogger()->warn("Frames {} and {} have different types", Id(), reference_keyframe->Id());
    return false;
  }
  auto reference_kf = dynamic_cast<const MonocularKeyFrame *>(reference_keyframe);

  // Ensure bows are computed
  ComputeBow();

  std::unordered_map<std::size_t, std::size_t> matches;
  ComputeMatchesFromReferenceKF(reference_kf, matches, false, true);

  logging::RetrieveLogger()->info("TWRKF: SNNMatcher returned {} matches for frames {} and {}",
                                  matches.size(),
                                  Id(),
                                  reference_keyframe->Id());

  if (matches.size() < 15) {
    return false;
  }

  for (const auto & match: matches) {
    auto map_point = reference_kf->map_points_.find(match.second);
    assert(map_point != reference_kf->map_points_.end());
    AddMapPoint(map_point->second, match.first);
  }

  OptimizePose();

  cv::imshow("TWRKF", debug::DrawMatches(GetFilename(),
                                         reference_keyframe->GetFilename(),
                                         matches,
                                         features_,
                                         reference_kf->GetFeatures()));

  return matches.size() >= 10;
}

bool MonocularFrame::IsVisible(map::MapPoint * map_point,
                               VisibleMapPoint & out_map_point,
                               precision_t radius_multiplier,
                               unsigned int window_size) const {
  return BaseMonocular::IsVisible(map_point,
                                  out_map_point,
                                  radius_multiplier,
                                  window_size,
                                  -1,
                                  this->GetPosition(),
                                  this->GetInversePosition(),
                                  feature_extractor_);
}

size_t MonocularFrame::GetMapPointCount() const {
  return map_points_.size();
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

KeyFrame * MonocularFrame::CreateKeyFrame() {
  return reference_keyframe_ = new MonocularKeyFrame(this);
}

void MonocularFrame::ListMapPoints(BaseFrame::MapPointSet & out_map_points) const {
  BaseMonocular::ListMapPoints(out_map_points);
}

bool MonocularFrame::ComputeMatchesForLinking(MonocularFrame * from_frame,
                                              unordered_map<size_t, size_t> & out_matches) const {
  features::matching::SNNMatcher<features::matching::iterators::AreaToIterator> matcher(0.9, 50);
  features::matching::iterators::AreaToIterator begin(0, &GetFeatures(), &from_frame->GetFeatures(), 100);
  features::matching::iterators::AreaToIterator
      end(GetFeatures().Size(), &GetFeatures(), &from_frame->GetFeatures(), 100);

  matcher.MatchWithIterators(begin, end, feature_extractor_, out_matches);
  logging::RetrieveLogger()->debug("Orientation validator discarded {} matches",
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
//    std::cout << point.second.x() << " " << point.second.y() << " " << point.second.z() << std::endl;

    precision_t max_invariance_distance, min_invariance_distance;
    feature_extractor_->ComputeInvariantDistances(GetPosition().Transform(point.second),
                                                  GetFeatures().keypoints[point.first],
                                                  max_invariance_distance,
                                                  min_invariance_distance);
    auto map_point = new map::MapPoint(point.second, Id(), max_invariance_distance, min_invariance_distance);
    AddMapPoint(map_point, point.first);
    from_frame->AddMapPoint(map_point, matches.find(point.first)->second);
    out_map_points.insert(map_point);
  }
}

void MonocularFrame::ComputeMatchesFromReferenceKF(const MonocularKeyFrame * reference_kf,
                                                   std::unordered_map<std::size_t, std::size_t> & out_matches,
                                                   bool self_keypoint_exists,
                                                   bool reference_kf_keypoint_exists) const {
  typedef features::matching::SNNMatcher<features::matching::iterators::BowToIterator> BOW_MATCHER;
  BOW_MATCHER bow_matcher(0.7, 50);
  features::matching::iterators::BowToIterator bow_it_begin(features_.bow_container.feature_vector.begin(),
                                                            &features_.bow_container.feature_vector,
                                                            &reference_kf->features_.bow_container.feature_vector,
                                                            &features_,
                                                            &reference_kf->features_,
                                                            &map_points_,
                                                            &reference_kf->map_points_,
                                                            self_keypoint_exists,
                                                            reference_kf_keypoint_exists);

  features::matching::iterators::BowToIterator bow_it_end(features_.bow_container.feature_vector.end(),
                                                          &features_.bow_container.feature_vector,
                                                          &reference_kf->features_.bow_container.feature_vector,
                                                          &features_,
                                                          &reference_kf->features_,
                                                          &map_points_,
                                                          &reference_kf->map_points_,
                                                          self_keypoint_exists,
                                                          reference_kf_keypoint_exists);

  bow_matcher.MatchWithIterators(bow_it_begin, bow_it_end, feature_extractor_, out_matches);
  features::matching::OrientationValidator
      (features_.keypoints, reference_kf->features_.keypoints).Validate(out_matches);

}

bool MonocularFrame::IsValid() const {
  return GetFeatures().Size() > 0;
}

void MonocularFrame::ComputeBow() {
  BaseMonocular::ComputeBow();
}

void MonocularFrame::OptimizePose() {
  optimization::OptimizePose(this);
}

void MonocularFrame::FilterVisibleMapPoints(const MapPointSet & map_points,
                                            std::list<VisibleMapPoint> & out_filetered_map_points,
                                            precision_t radius_multiplier,
                                            unsigned int window_size) const {
  VisibleMapPoint map_point;
  for (auto mp: map_points) {
    if (IsVisible(mp, map_point, radius_multiplier, window_size))
      out_filetered_map_points.push_back(map_point);
  }

}

void MonocularFrame::SearchInVisiblePoints(const list<VisibleMapPoint> & filtered_map_points,
                                           precision_t matcher_snn_threshold) {
  auto map_points = GetMapPoints();
  features::matching::iterators::ProjectionSearchIterator begin
      (filtered_map_points.begin(),
       filtered_map_points.end(),
       &features_,
       &map_points);

  features::matching::iterators::ProjectionSearchIterator end
      (filtered_map_points.end(),
       filtered_map_points.end(),
       &features_,
       &map_points);

  typedef features::matching::SNNMatcher<features::matching::iterators::ProjectionSearchIterator> Matcher;
  Matcher matcher(matcher_snn_threshold, constants::MONO_TWMM_THRESHOLD_HIGH);
  Matcher::MatchMapType matches;
  matcher.MatchWithIterators(begin, end, feature_extractor_, matches);
  logging::RetrieveLogger()->debug("SLMM: SNN matcher found {} matches", matches.size());
  for (auto & match: matches) {
    AddMapPoint(match.first, match.second);
  }
}

void MonocularFrame::SearchInVisiblePoints(const std::list<VisibleMapPoint> & filtered_map_points) {
  ComputeBow();
  SearchInVisiblePoints(filtered_map_points, 0.8);
}

void MonocularFrame::UpdateFromReferenceKeyFrame() {
  if (reference_keyframe_) {
    SetPosition(reference_keyframe_->GetPosition());
    map_points_ = reference_keyframe_->GetMapPoints();
  }
}

void MonocularFrame::FilterFromLastFrame(MonocularFrame * last_frame,
                                         std::list<VisibleMapPoint> & out_visibles,
                                         precision_t radius_multiplier) {
  geometry::Pose pose = GetPosition();
  geometry::Pose inverse_pose = GetInversePosition();
  auto mps = last_frame->GetMapPoints();
  VisibleMapPoint vmp;
  std::list<VisibleMapPoint> visibles;
  for (auto mp: mps) {
    if (BaseMonocular::IsVisible(mp.second,
                                 vmp,
                                 radius_multiplier,
                                 0,
                                 last_frame->features_.keypoints[mp.first].level,
                                 pose,
                                 inverse_pose,
                                 feature_extractor_)) {
      out_visibles.push_back(vmp);
    }
  }
}

BaseMonocular::MonocularMapPoints MonocularFrame::GetBadMapPoints() {

  BaseMonocular::MonocularMapPoints result;
  for (auto mp: map_points_)
    if (mp.second->IsBad())
      result.insert(mp);

  return result;
}

bool MonocularFrame::EstimatePositionByProjectingMapPoints(Frame * frame, list<VisibleMapPoint> & out_visibles) {

  auto last_frame = dynamic_cast<MonocularFrame *>(frame);
  precision_t radiuses[] = {15, 30};

  for (precision_t radius: radiuses) {
    map_points_.clear();
    out_visibles.clear();
    FilterFromLastFrame(last_frame, out_visibles, radius);
    SearchInVisiblePoints(out_visibles, 0.9);
    if (map_points_.size() >= 20) {
      OptimizePose();
      if (map_points_.size() >= 10)
        return true;
    }
  }
  map_points_.clear();
  return false;
}

}
}
}