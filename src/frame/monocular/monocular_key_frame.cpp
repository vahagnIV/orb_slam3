//
// Created by vahagn on 11/05/2021.
//

#include "monocular_key_frame.h"
#include "monocular_frame.h"
#include <map/map_point.h>
#include <map/map.h>
#include <map/atlas.h>
#include <logging.h>
#include <features/handlers/DBoW2/bow_to_iterator.h>
#include <features/matching/iterators/projection_search_iterator.h>
#include <features/matching/second_nearest_neighbor_matcher.hpp>
#include <geometry/utils.h>
#include <geometry/ransac_sim3_solver.h>
#include <optimization/monocular_optimization.h>

namespace orb_slam3 {
namespace frame {
namespace monocular {

MonocularKeyFrame::MonocularKeyFrame(std::istream &istream, serialization::SerializationContext &context) :
    KeyFrame(istream, context),
    BaseMonocular(istream, context) {
}

MonocularKeyFrame::MonocularKeyFrame(MonocularFrame * frame) : KeyFrame(frame->GetTimeCreated(),
                                                                        frame->GetFilename(),
                                                                        frame->GetSensorConstants(),
                                                                        frame->Id(),
                                                                        frame->GetAtlas(),
                                                                        frame->GetMap(),
                                                                        frame->GetPosition()),
                                                               BaseMonocular(*frame),
                                                               map_points_mutex_() {
  SetFeatureHandler(frame->GetFeatureHandler());
  SetMap(frame->GetMap());
}

FrameType MonocularKeyFrame::Type() const {
  return MONOCULAR;
}

void MonocularKeyFrame::SetCamera(const camera::ICamera * icamera) {
  if(icamera->Type() != camera::CameraType::MONOCULAR)
    throw std::runtime_error("Invalid camera for monocular frame");

  BaseMonocular::SetCamera(dynamic_cast<const camera::MonocularCamera *>(icamera));
}

BaseMonocular::MonocularMapPoints MonocularKeyFrame::GetMapPointsWithLock() const {
  std::unique_lock<std::recursive_mutex> lock(map_points_mutex_);
  return GetMapPoints();
}

void MonocularKeyFrame::ListMapPoints(BaseFrame::MapPointSet & out_map_points) const {
  BaseMonocular::ListMapPoints(out_map_points);
}

TVector3D MonocularKeyFrame::GetNormal(const TPoint3D & point) const {
  TPoint3D normal = GetInversePosition().T - point;
//  normal.normalize();
  return normal;
}

TVector3D MonocularKeyFrame::GetNormalFromStaging(const TPoint3D & point) const {
  TPoint3D normal = GetStagingPosition().GetInversePose().T - point;
//  normal.normalize();
  return normal;
}

precision_t MonocularKeyFrame::ComputeBaseline(const geometry::Pose & local_pose,
                                               const geometry::Pose & others_pose) {
  auto inv_local_pose = local_pose.GetInversePose();
  auto inv_other_pose = others_pose.GetInversePose();
  auto baseline_vector = inv_local_pose.T - inv_other_pose.T;
  return baseline_vector.norm();
}

bool MonocularKeyFrame::BaseLineIsEnough(const MapPointSet & others_map_points,
                                         const geometry::Pose & local_pose,
                                         const geometry::Pose & others_pose) {

  precision_t median_depth = ComputeSceneMedianDepth(others_map_points, 2, others_pose);
  precision_t baseline = ComputeBaseline(local_pose, others_pose);
  return baseline / median_depth > 0.01;
}

precision_t MonocularKeyFrame::ComputeSceneMedianDepth(const MapPointSet & map_points,
                                                       unsigned int q,
                                                       const geometry::Pose & pose) {

  std::vector<precision_t> depths;
  depths.reserve(map_points.size());
  TVector3D R_z = pose.R.row(2);

  for (auto mp: map_points) {
    if (mp->IsBad())
      continue;
    depths.push_back(R_z.dot(mp->GetPosition()) + pose.T.z());
  }

  return depths[(depths.size() - 1) / q];
}

void MonocularKeyFrame::CreateNewMapPoints(frame::KeyFrame * other, NewMapPoints & out_newly_created) const {

  if (other->Type() != Type()) {
    throw std::runtime_error("Matching Stereo With Monocular is not implemented yet");
  }

  auto other_frame = dynamic_cast<MonocularKeyFrame *> (other);
  if (other_frame == this)
    return;

  const MonocularMapPoints & local_map_points_map = GetMapPoints();
  const MonocularMapPoints & others_map_points_map = other_frame->GetMapPoints();

  MapPointSet local_map_points, others_map_points;
  MapToSet(local_map_points_map, local_map_points);
  MapToSet(others_map_points_map, others_map_points);

  auto local_pose = GetPosition();
  auto other_pose = other_frame->GetPosition();

  if (!BaseLineIsEnough(others_map_points, local_pose, other_pose)) {
    logging::RetrieveLogger()->debug("Baseline between frames {} and {} is not enough", Id(), other->Id());
    return;
  }

  logging::RetrieveLogger()->debug("LM: Initial local map point count: {}", local_map_points.size());

  features::FastMatches matches;
  feature_handler_->FastMatch(other_frame->GetFeatureHandler(), matches, features::MatchingSeverity::STRONG, false);

  geometry::Pose relative_pose;
  geometry::utils::ComputeRelativeTransformation(GetPosition(), other_frame->GetPosition(), relative_pose);

  logging::RetrieveLogger()->debug("LM: SNNMatcher found {} matches between {} and {}",
                                   matches.size(),
                                   other_frame->Id(),
                                   Id());

  for (auto match: matches) {
    if (local_map_points_map.find(match.first) != local_map_points_map.end()
        || others_map_points_map.find(match.second) != others_map_points_map.end())
      continue;
    precision_t parallax;
    TPoint3D triangulated;
    if (!geometry::utils::TriangulateAndValidate(other_frame->feature_handler_->GetFeatures().undistorted_and_unprojected_keypoints[match.second],
                                                 feature_handler_->GetFeatures().undistorted_and_unprojected_keypoints[match.first],
                                                 relative_pose,
                                                 5.991 * GetMonoCamera()->FxInv() * GetMonoCamera()->FxInv(),
                                                 5.991 * other_frame->GetMonoCamera()->FxInv()
                                                     * other_frame->GetMonoCamera()->FxInv(),
                                                 constants::PARALLAX_THRESHOLD,
                                                 parallax,
                                                 triangulated))
      continue;
    TPoint2D pt_other, pt_this;
    other_frame->GetMonoCamera()->ProjectAndDistort(triangulated, pt_other);
    GetMonoCamera()->ProjectAndDistort(relative_pose.Transform(triangulated), pt_this);
    if (!other_frame->GetMonoCamera()->IsInFrustum(pt_other))
      continue;
    if (!GetMonoCamera()->IsInFrustum(pt_this))
      continue;
    precision_t min_invariance_distance, max_invariance_distance;
    feature_handler_->GetFeatureExtractor()->ComputeInvariantDistances(triangulated,
                                                                       other_frame->feature_handler_->GetFeatures().keypoints[match.second],
                                                                       max_invariance_distance,
                                                                       min_invariance_distance);
    auto map_point = new map::MapPoint(other_frame->GetInversePosition().Transform(triangulated),
                                       Id(),
                                       max_invariance_distance,
                                       min_invariance_distance,
                                       GetMap());

    out_newly_created.emplace_back(Observation(map_point, const_cast<MonocularKeyFrame *>(this), match.first),
                                   Observation(map_point, other, match.second));
  }
}

void MonocularKeyFrame::MatchVisibleMapPoints(const std::list<MapPointVisibilityParams> & visibles,
                                              std::list<std::pair<map::MapPoint *,
                                                                  map::MapPoint *>> & out_matched_map_points,
                                              std::list<Observation> & out_local_matches) const {

  typedef features::matching::iterators::ProjectionSearchIterator IteratorType;
  IteratorType begin(visibles.begin(), visibles.end(), &feature_handler_->GetFeatures());
  IteratorType end(visibles.end(), visibles.end(), &feature_handler_->GetFeatures());
  typedef features::matching::SNNMatcher<IteratorType> MatcherType;
  MatcherType::MatchMapType matches;
  MatcherType matcher(1.0, GetMap()->GetAtlas()->GetFeatureExtractor()->GetLowThreshold());
  matcher.MatchWithIterators(begin, end, feature_handler_->GetFeatureExtractor(), matches);

  for (auto & match: matches) {

    map::MapPoint *local_mp = GetMapPoint(match.second);
    const features::KeyPoint &key_point = GetFeatureHandler()->GetFeatures().keypoints[match.second];
    const TPoint2D &original_point = key_point.pt;
    TPoint2D projected;
    GetMonoCamera()->ProjectAndDistort(GetPosition().Transform(match.first->GetPosition()), projected);
    precision_t error = (original_point - projected).squaredNorm();
    if (error / GetMap()->GetAtlas()->GetFeatureExtractor()->GetAcceptableSquareError(key_point.level)
        > 5.99)
      continue;

    if (nullptr == local_mp) {
      if (feature_handler_->GetFeatures().undistorted_and_unprojected_keypoints[match.second].z() > 0)
        out_local_matches.emplace_back(match.first, const_cast<MonocularKeyFrame *>(this), match.second);
    } else {
      out_matched_map_points.emplace_back(local_mp, match.first);
    }
  }

}

void MonocularKeyFrame::SetMap(map::Map * map) {
  if (map_)
    map_->EraseKeyFrame(this);
  BaseFrame::SetMap(map);
}

void MonocularKeyFrame::FilterVisibleMapPoints(const BaseFrame::MapPointSet & map_points,
                                               std::list<MapPointVisibilityParams> & out_visibles,
                                               bool use_staging) const {
  MapPointVisibilityParams visible_map_point;
  MapPointSet local_map_points;
  ListMapPoints(local_map_points);

  auto pose = use_staging ? GetStagingPosition() : GetPosition();
  auto inverse_pose = pose.GetInversePose();
  for (auto mp: map_points) {
    if (mp->IsBad()) continue;
    if (!mp->IsInKeyFrame(this)) {
      visible_map_point.map_point = mp;
      TPoint3D map_point_position = use_staging ? mp->GetStagingPosition() : mp->GetPosition();
      if (local_map_points.find(mp) == local_map_points.end() &&
          BaseMonocular::PointVisible(pose.Transform(map_point_position),
                                      use_staging ? mp->GetStagingPosition() : mp->GetPosition(),
                                      use_staging ? mp->GetStagingMinInvarianceDistance()
                                                  : mp->GetMinInvarianceDistance(),
                                      use_staging ? mp->GetStagingMaxInvarianceDistance()
                                                  : mp->GetMaxInvarianceDistance(),
                                      use_staging ? mp->GetStagingNormal() : mp->GetNormal(),
                                      inverse_pose.T,
                                      GetSensorConstants()->max_allowed_discrepancy,
                                      -1,
                                      visible_map_point,
                                      GetMap()->GetAtlas()->GetFeatureExtractor()))
        out_visibles.push_back(visible_map_point);
    }
  }
}

void MonocularKeyFrame::EraseMapPointImpl(Observation & observation) {
  BaseMonocular::EraseMapPoint(observation.GetFeatureId());
}

map::MapPoint * MonocularKeyFrame::EraseMapPoint(size_t feature_id) {
  map::MapPoint * mp = BaseMonocular::EraseMapPoint(feature_id);
  Observation observation;
  if (mp->GetObservation(this, observation)) {
    mp->EraseObservation(this);
  }
  return mp;
}

void MonocularKeyFrame::SetBad() {
  KeyFrame::SetBad();
  MonocularMapPoints mps = GetMapPoints();
  for (auto mp: mps) {
    EraseMapPoint(mp.first);
  }
}

void MonocularKeyFrame::FindMatchingMapPoints(const KeyFrame * other,
                                              MapPointMatches & out_matches) const {

  features::FastMatches matches;
  auto mono_other = dynamic_cast<const MonocularKeyFrame *>(other);
  assert(nullptr != mono_other);
  feature_handler_->FastMatch(mono_other->GetFeatureHandler(),
                              matches,
                              features::MatchingSeverity::WEAK,
                              true);

  for (const auto & match: matches) {

    auto local_mp_it = GetMapPoints().find(match.first);
    if (local_mp_it == GetMapPoints().end()) continue;
    map::MapPoint * local_map_point = local_mp_it->second;

    auto kf_map_point_it = mono_other->GetMapPoints().find(match.second);
    if (kf_map_point_it == mono_other->GetMapPoints().end()) continue;
    map::MapPoint * kf_map_point = kf_map_point_it->second;
    if (!local_map_point->IsBad() && !kf_map_point->IsBad())
      out_matches.emplace_back(local_map_point, kf_map_point);
  }

}

int MonocularKeyFrame::GetMapPointLevel(const map::MapPoint * map_point) const {
  auto obs = map_point->Observations();
  const auto mp_obs = obs.find(const_cast<MonocularKeyFrame * const>(this));
  auto handler = GetFeatureHandler();
  if (mp_obs != map_point->Observations().end()) {
    const auto & features = handler->GetFeatures();
    size_t feature_id = mp_obs->second.GetFeatureId();
    return features.keypoints[feature_id].level;
  } else
    return
        handler->GetFeatureExtractor()->PredictScale(GetPosition().Transform(map_point->GetPosition()).norm(),
                                                     map_point->GetMaxInvarianceDistance() / 1.2);
}

void MonocularKeyFrame::FilterVisibleMapPoints(const MapPointSet &map_points,
                                               const geometry::Sim3Transformation &relative_transformation,
                                               const geometry::Pose &mp_local_transformation,
                                               precision_t radius_multiplier,
                                               std::list<MapPointVisibilityParams> &out_visibles) const {
  MapPointVisibilityParams tmp;
  geometry::Pose pose = GetPosition();
  geometry::Pose inverse_pose = pose.GetInversePose();

  for (const auto & mp: map_points) {
    if (mp->IsBad())
      continue;

    tmp.map_point = mp;
    TPoint3D mp_local_in_kf = mp_local_transformation.Transform(mp->GetPosition());
    TPoint3D mp_local_coords = relative_transformation.Transform(mp_local_in_kf);
    TPoint3D mp_world_pos = inverse_pose.Transform(mp_local_coords);
    TVector3D normal_w = mp->GetNormal();
    normal_w = mp_local_transformation.Transform(normal_w);
    normal_w = relative_transformation.Transform(normal_w);
    normal_w = inverse_pose.Transform(normal_w);

    if (BaseMonocular::PointVisible(mp_local_coords,
                                    mp_world_pos,
                                    relative_transformation.s * mp->GetMinInvarianceDistance(),
                                    relative_transformation.s * mp->GetMaxInvarianceDistance(),
                                    normal_w,
                                    inverse_pose.T,
                                    radius_multiplier,
                                    -1,
                                    tmp,
                                    GetMap()->GetAtlas()->GetFeatureExtractor()));
    out_visibles.push_back(tmp);
  }
}

size_t MonocularKeyFrame::AdjustSim3Transformation(std::list<MapPointVisibilityParams> & visibles,
                                                   const KeyFrame * relative_kf,
                                                   geometry::Sim3Transformation & in_out_transformation) const {
  typedef features::matching::iterators::ProjectionSearchIterator ProjectionSearchIterator;
  typedef features::matching::SNNMatcher<ProjectionSearchIterator> TMatcher;
  ProjectionSearchIterator begin(visibles.begin(), visibles.end(), &feature_handler_->GetFeatures());
  ProjectionSearchIterator end(visibles.end(), visibles.end(), &feature_handler_->GetFeatures());
  TMatcher matcher(0.9, 50);
  TMatcher::MatchMapType matches;
  matcher.MatchWithIterators(begin, end, GetMap()->GetAtlas()->GetFeatureExtractor(), matches);
//  std::cout << "Match count: " << matches.size() << std::endl;
  if (matches.size() < 50)
    return 0;

  std::unordered_map<map::MapPoint *, int> levels;
  for (auto visible: visibles) {
    levels[visible.map_point] = visible.level;
  }

  auto mono_rel_kf = dynamic_cast<const MonocularKeyFrame *>(relative_kf);
  assert(nullptr != mono_rel_kf);

  return optimization::OptimizeSim3(this, mono_rel_kf, in_out_transformation, matches, levels);
}

bool MonocularKeyFrame::FindSim3Transformation(const KeyFrame::MapPointMatches & map_point_matches,
                                               const KeyFrame * loop_candidate,
                                               geometry::Sim3Transformation & out_transormation) const {
  std::vector<std::pair<TPoint3D, TPoint3D>> matched_points(map_point_matches.size());
  std::vector<std::pair<TPoint2D, TPoint2D>> matched_point_projections(map_point_matches.size());
  auto mono_loop_cand = dynamic_cast<const MonocularKeyFrame *>(loop_candidate);
  assert(nullptr != mono_loop_cand);

  const geometry::Pose kf_pose = GetPosition();
  const geometry::Pose loop_candidate_pose = loop_candidate->GetPosition();

  std::transform(map_point_matches.begin(),
                 map_point_matches.end(),
                 matched_points.begin(),
                 [&kf_pose, &loop_candidate_pose](const std::pair<map::MapPoint *, map::MapPoint *> & pair) {
                   return std::pair<TPoint3D, TPoint3D>(kf_pose.Transform(pair.first->GetPosition()),
                                                        loop_candidate_pose.Transform(pair.second->GetPosition()));
                 });

  const camera::MonocularCamera * local_camera = GetMonoCamera();
  const camera::MonocularCamera
      * loop_candidate_camera = mono_loop_cand->GetMonoCamera();

  std::transform(matched_points.begin(),
                 matched_points.end(),
                 matched_point_projections.begin(),
                 [& local_camera, &loop_candidate_camera]
                     (const std::pair<TPoint3D, TPoint3D> & match) {
                   TPoint2D local_projection, loop_candidate_projection;
                   local_camera->ProjectAndDistort(match.first, local_projection);
                   loop_candidate_camera->ProjectAndDistort(match.second,
                                                            loop_candidate_projection);
                   return std::make_pair(local_projection, loop_candidate_projection);
                 });

  std::vector<std::pair<precision_t, precision_t>> errors;
  for (const auto & mps: map_point_matches) {
    const auto & local_mp = mps.first;
    const auto & remote_mp = mps.second;

    precision_t error1, error2;

    error1 = 9.210 * GetFeatureHandler()->GetFeatureExtractor()->GetAcceptableSquareError(GetMapPointLevel(local_mp));
    error2 =
        9.210
            * loop_candidate->GetFeatureHandler()->GetFeatureExtractor()->GetAcceptableSquareError(mono_loop_cand->GetMapPointLevel(
                remote_mp));
    errors.emplace_back(error1, error2);
  }
  geometry::RANSACSim3Solver
      solver(&matched_points, &matched_point_projections, local_camera, loop_candidate_camera, &errors, 300);
  return solver(out_transormation);
}

void MonocularKeyFrame::InitializeImpl() {

  for (auto mp: GetMapPoints()) {
    if (mp.second->IsBad())
      continue;
    mp.second->AddObservation(Observation(mp.second, this, mp.first));
    mp.second->ApplyStaging();
  }
  logging::RetrieveLogger()->debug("Created new keyframe with id {}", Id());
  logging::RetrieveLogger()->debug("Number of map points:  {}", GetMapPointsCount());

  covisibility_graph_.Update();
}

void MonocularKeyFrame::LockMapPointContainer() const {
  map_points_mutex_.lock();
}

void MonocularKeyFrame::UnlockMapPointContainer() const {
  map_points_mutex_.unlock();
}

void MonocularKeyFrame::AddMapPointImpl(Observation & observation) {
  BaseMonocular::AddMapPoint(observation.GetMapPoint(), observation.GetFeatureId());
}

int MonocularKeyFrame::GetScaleLevel(const map::MapPoint * map_point) const {
  Observation observation;
  if (!map_point->GetObservation(this, observation))
    return -1;
  return GetScaleLevel(observation);
}

int MonocularKeyFrame::GetScaleLevel(const Observation &observation) const {
  return GetFeatureHandler()->GetFeatures().keypoints[observation.GetFeatureId()].level;
}

const camera::ICamera *MonocularKeyFrame::GetCamera() const {
  return this->GetMonoCamera();
}

void MonocularKeyFrame::SerializeToStream(std::ostream &stream) const {
  KeyFrame::SerializeToStream(stream);
  BaseMonocular::SerializeToStream(stream);
}

}
}
}
