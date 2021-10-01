//
// Created by vahagn on 11/05/2021.
//

#include "monocular_frame.h"
#include <logging.h>
#include <features/matching/second_nearest_neighbor_matcher.hpp>
#include <features/matching/iterators/area_to_iterator.h>
#include <features/matching/iterators/projection_search_iterator.h>
#include <features/matching/validators/orientation_validator.h>
#include <features/ifeature_extractor.h>
#include <map/map_point.h>
#include <geometry/two_view_reconstructor.h>
#include <optimization/monocular_optimization.h>
#include <src/features/handlers/DBoW2/bow_to_iterator.h>
#include "monocular_key_frame.h"
#include <map/map.h>
#include <debug/debug_utils.h>
#include <serialization/serialization_context.h>

namespace orb_slam3 {
namespace frame {
namespace monocular {

MonocularFrame::MonocularFrame(const TImageGray8U & image,
                               TimePoint time_point,
                               const std::string & filename,
                               const features::IFeatureExtractor * feature_extractor,
                               const camera::MonocularCamera * camera,
                               const SensorConstants * sensor_constants,
                               const features::HandlerFactory * handler_factory) : Frame(time_point,
                                                                                         filename,
                                                                                         sensor_constants),
                                                                                   BaseMonocular(camera),
                                                                                   reference_keyframe_(nullptr) {
  features::Features features(camera->Width(), camera->Height());
  feature_extractor->Extract(image, features);
  features.AssignFeaturesToGrid();

  features.undistorted_keypoints.resize(features.Size());
  features.undistorted_and_unprojected_keypoints.resize(features.Size());
  for (size_t i = 0; i < features.Size(); ++i) {
    camera->UndistortPoint(features.keypoints[i].pt, features.undistorted_keypoints[i]);
    camera->UnprojectAndUndistort(features.keypoints[i].pt, features.undistorted_and_unprojected_keypoints[i]);
  }

  feature_handler_ = handler_factory->CreateFeatureHandler(features, feature_extractor);
}

MonocularFrame::MonocularFrame(std::istream &stream, serialization::SerializationContext &context)
    : Frame(stream, context),
      BaseMonocular(stream, context) {
  size_t reference_kf_id;
  READ_FROM_STREAM(reference_kf_id, stream);
  frame::KeyFrame *rf_kf = reference_kf_id ? context.kf_id[reference_kf_id] : nullptr;
  if (rf_kf->Type() != Type())
    throw std::runtime_error(
        "Invalid reference kf type while deserializing monocular frame. Only monocular keyframes are supported");
  reference_keyframe_ = dynamic_cast<MonocularKeyFrame *> (rf_kf);
  assert(nullptr != reference_keyframe_);
  size_t mp_count;
  READ_FROM_STREAM(mp_count, stream);
  for (size_t i = 0; i < mp_count; ++i) {
    size_t feature_id, mp_id;
    READ_FROM_STREAM(feature_id, stream);
    READ_FROM_STREAM(mp_id, stream);
    AddMapPoint(context.mp_id[mp_id], feature_id);
  }
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

  geometry::TwoViewReconstructor reconstructor(200, GetMonoCamera()->FxInv());
  std::unordered_map<size_t, TPoint3D> points;
  geometry::Pose pose;
  if (!reconstructor.Reconstruct(feature_handler_->GetFeatures().undistorted_and_unprojected_keypoints,
                                 from_frame->feature_handler_->GetFeatures().undistorted_and_unprojected_keypoints,
                                 matches,
                                 pose,
                                 points)) {

    logging::RetrieveLogger()->info("LINKING: Could not reconstruct points between frames {} and {}. Skipping...",
                                    Id(),
                                    other->Id());
    return false;
  }
  for (auto it = matches.begin(); it != matches.end();) {
    if (points.find(it->first) == points.end()) {
      it = matches.erase(it);
    } else
      ++it;
  }
  cv::imshow("Linking",
             debug::DrawMatches(GetFilename(),
                                other->GetFilename(),
                                matches,
                                feature_handler_->GetFeatures(),
                                from_frame->feature_handler_->GetFeatures()));
  cv::waitKey();

  this->SetStagingPosition(pose);
  this->ApplyStaging();
  std::unordered_set<map::MapPoint *> map_points;
  InitializeMapPointsFromMatches(matches, points, from_frame, map_points);
  logging::RetrieveLogger()->debug("Linking found {} matches", GetMapPointsCount());
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

  std::unordered_map<std::size_t, std::size_t> matches;
  ComputeMatchesFromReferenceKF(reference_kf, matches);

  logging::RetrieveLogger()->info("TWRKF: SNNMatcher returned {} matches for frames {} and {}",
                                  matches.size(),
                                  Id(),
                                  reference_keyframe->Id());

  if (matches.size() < 15) {
    return false;
  }

  MonocularMapPoints reference_kf_map_points = reference_kf->GetMapPointsWithLock();
  for (const auto & match: matches) {
    auto map_point = reference_kf_map_points.find(match.second);
    assert(map_point != reference_kf_map_points.end());
    AddMapPoint(map_point->second, match.first);
  }

  OptimizePose();

  cv::imshow("TWRKF", debug::DrawMatches(GetFilename(),
                                         reference_keyframe->GetFilename(),
                                         matches,
                                         feature_handler_->GetFeatures(),
                                         reference_kf->feature_handler_->GetFeatures()));

  return matches.size() >= 10;
}

size_t MonocularFrame::GetMapPointsCount() const {
  return BaseMonocular::GetMapPointsCount();
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

KeyFrame * MonocularFrame::CreateKeyFrame() {
  reference_keyframe_ = new MonocularKeyFrame(this);
  map_->AddKeyFrame(reference_keyframe_);
  return reference_keyframe_;
}

void MonocularFrame::ListMapPoints(BaseFrame::MapPointSet & out_map_points) const {
  BaseMonocular::ListMapPoints(out_map_points);
}

bool MonocularFrame::ComputeMatchesForLinking(MonocularFrame * from_frame,
                                              std::unordered_map<size_t, size_t> & out_matches) const {
  features::matching::SNNMatcher<features::matching::iterators::AreaToIterator> matcher(0.9, 50);
  features::matching::iterators::AreaToIterator
      begin(0, &feature_handler_->GetFeatures(), &from_frame->feature_handler_->GetFeatures(), 100);
  features::matching::iterators::AreaToIterator
      end(feature_handler_->GetFeatures().Size(),
          &feature_handler_->GetFeatures(),
          &from_frame->feature_handler_->GetFeatures(),
          100);

  matcher.MatchWithIterators(begin, end, feature_handler_->GetFeatureExtractor(), out_matches);
  logging::RetrieveLogger()->debug("Orientation validator discarded {} matches",
                                   features::matching::OrientationValidator
                                       (feature_handler_->GetFeatures().keypoints,
                                        from_frame->feature_handler_->GetFeatures().keypoints).Validate(
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
  for (const auto & point: points) {
//    std::cout << point.second.x() << " " << point.second.y() << " " << point.second.z() << std::endl;

    precision_t max_invariance_distance, min_invariance_distance;
    feature_handler_->GetFeatureExtractor()->ComputeInvariantDistances(GetPosition().Transform(point.second),
                                                                       feature_handler_->GetFeatures().keypoints[point.first],
                                                                       max_invariance_distance,
                                                                       min_invariance_distance);
    auto map_point = new map::MapPoint(point.second, Id(), max_invariance_distance, min_invariance_distance, GetMap());
    AddMapPoint(map_point, point.first);
    from_frame->AddMapPoint(map_point, matches.find(point.first)->second);
    out_map_points.insert(map_point);
  }
}

void MonocularFrame::ComputeMatchesFromReferenceKF(const MonocularKeyFrame * reference_kf,
                                                   std::unordered_map<std::size_t, std::size_t> & out_matches) const {
  feature_handler_->FastMatch(reference_kf->GetFeatureHandler(),
                              out_matches,
                              features::MatchingSeverity::MIDDLE,
                              true);

  MonocularMapPoints reference_kf_map_points = reference_kf->GetMapPointsWithLock();
  auto match_it = out_matches.begin();
  while (match_it != out_matches.end()) {
    if (reference_kf_map_points.find(match_it->second) == reference_kf_map_points.end())
      match_it = out_matches.erase(match_it);
    else
      ++match_it;
  }
}

bool MonocularFrame::IsValid() const {
  return feature_handler_->GetFeatures().Size() > 0;
}

void MonocularFrame::OptimizePose() {
  optimization::OptimizePose(this);
  this->ApplyStaging();
}

void MonocularFrame::FilterVisibleMapPoints(const MapPointSet & map_points,
                                            std::list<MapPointVisibilityParams> & out_filetered_map_points,
                                            precision_t radius_multiplier) const {
  MapPointVisibilityParams map_point;
  geometry::Pose pose = GetPosition();
  geometry::Pose inverse_pose = pose.GetInversePose();
  for (auto mp: map_points) {
    map_point.map_point = mp;
    TPoint3D position = mp->GetPositionWithLock();
    TVector3D normal = mp->GetNormalWithLock();
    if (BaseMonocular::PointVisible(pose.Transform(position),
                                    position,
                                    mp->GetMinInvarianceDistance(),
                                    mp->GetMaxInvarianceDistance(),
                                    normal,
                                    inverse_pose.T,
                                    radius_multiplier,
                                    -1,
                                    map_point,
                                    GetFeatureExtractor())) {

      out_filetered_map_points.push_back(map_point);
    }
    /*if (IsVisible(mp, map_point, radius_multiplier)) {
      out_filetered_map_points.push_back(map_point);
    }*/
  }

}

void MonocularFrame::SearchInVisiblePoints(const std::list<MapPointVisibilityParams> & filtered_map_points,
                                           precision_t matcher_snn_threshold) {

  features::matching::iterators::ProjectionSearchIterator begin
      (filtered_map_points.begin(),
       filtered_map_points.end(),
       &feature_handler_->GetFeatures());

  features::matching::iterators::ProjectionSearchIterator end
      (filtered_map_points.end(),
       filtered_map_points.end(),
       &feature_handler_->GetFeatures());

  typedef features::matching::SNNMatcher<features::matching::iterators::ProjectionSearchIterator> Matcher;
  Matcher matcher(matcher_snn_threshold, constants::MONO_TWMM_THRESHOLD_HIGH);
  Matcher::MatchMapType matches;
  matcher.MatchWithIterators(begin, end, feature_handler_->GetFeatureExtractor(), matches);
  logging::RetrieveLogger()->debug("SLMM: SNN matcher found {} matches", matches.size());
  for (auto & match: matches) {
    AddMapPoint(match.first, match.second);
  }
}

void MonocularFrame::SearchInVisiblePoints(const std::list<MapPointVisibilityParams> & filtered_map_points) {
  SearchInVisiblePoints(filtered_map_points, 0.8);
}

void MonocularFrame::UpdateFromReferenceKeyFrame() {
  if (reference_keyframe_) {
    SetStagingPosition(reference_keyframe_->GetPositionWithLock());
    ApplyStaging();
    ClearMapPoints();
    for (const auto & mp: reference_keyframe_->GetMapPointsWithLock()) {
      AddMapPoint(mp.second, mp.first);
    }
  }
}

void MonocularFrame::FilterFromLastFrame(MonocularFrame * last_frame,
                                         std::list<MapPointVisibilityParams> & out_visibles,
                                         precision_t radius_multiplier) {
  geometry::Pose pose = GetPosition();
  geometry::Pose inverse_pose = GetInversePosition();
  auto mps = last_frame->GetMapPoints();
  MapPointVisibilityParams vmp;
  std::list<MapPointVisibilityParams> visibles;
  for (auto mp: mps) {
    assert(mp.first < last_frame->feature_handler_->GetFeatures().keypoints.size());
    TPoint3D position = mp.second->GetPositionWithLock();
    TVector3D normal = mp.second->GetNormalWithLock();

    if (BaseMonocular::PointVisible(pose.Transform(position),
                                    position,
                                    mp.second->GetMinInvarianceDistance(),
                                    mp.second->GetMaxInvarianceDistance(),
                                    normal,
                                    inverse_pose.T,
                                    radius_multiplier,
                                    last_frame->feature_handler_->GetFeatures().keypoints[mp.first].level,
                                    vmp,
                                    GetFeatureExtractor())) {
      vmp.map_point = mp.second;
      out_visibles.push_back(vmp);
    }

  }
}

BaseMonocular::MonocularMapPoints MonocularFrame::GetBadMapPoints() const {
  BaseMonocular::MonocularMapPoints result;
  for (auto mp: GetMapPoints())
    if (mp.second->IsBad())
      result.insert(mp);
  return result;
}

bool MonocularFrame::EstimatePositionByProjectingMapPoints(Frame * frame,
                                                           std::list<MapPointVisibilityParams> & out_visibles) {

  auto last_frame = dynamic_cast<MonocularFrame *>(frame);
  precision_t radiuses[] = {15, 30};

  for (precision_t radius: radiuses) {
    ClearMapPoints();
    out_visibles.clear();
    FilterFromLastFrame(last_frame, out_visibles, radius);
    SearchInVisiblePoints(out_visibles, 0.9);
    if (GetMapPointsCount() >= 20) {
      OptimizePose();
      if (GetMapPointsCount() >= 10) {
        ApplyStaging();
        return true;
      }
    }
  }
  ClearMapPoints();
  return false;
}

const camera::ICamera * MonocularFrame::GetCamera() const {
  return this->GetMonoCamera();
}

void MonocularFrame::SetCamera(const camera::ICamera * icamera) {
  if (icamera->Type() != camera::CameraType::MONOCULAR)
    throw std::runtime_error("Invalid camera for monocular frame");

  BaseMonocular::SetCamera(dynamic_cast<const camera::MonocularCamera *>(icamera));
}

void MonocularFrame::SerializeToStream(std::ostream & stream) const {
  BaseMonocular::SerializeToStream(stream);
  size_t reference_kf_id = reference_keyframe_ ? reference_keyframe_->Id() : 0;
  WRITE_TO_STREAM(reference_kf_id, stream);
  size_t mp_count = GetMapPoints().size();
  WRITE_TO_STREAM(mp_count, stream);
  for (auto feature_mp: GetMapPoints()) {
    WRITE_TO_STREAM(feature_mp.first, stream);
    size_t mp_id = reinterpret_cast<size_t>(feature_mp.second);
    WRITE_TO_STREAM(mp_id, stream);
  }
}

}
}
}
