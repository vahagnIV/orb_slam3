//
// Created by vahagn on 11/05/2021.
//

#include "monocular_key_frame.h"
#include "monocular_frame.h"
#include <map/map_point.h>
#include <map/map.h>
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
#define WRITE_TO_STREAM(num, stream) stream.write((char *)(&num), sizeof(num));
MonocularKeyFrame::MonocularKeyFrame(MonocularFrame * frame) : KeyFrame(frame->GetTimeCreated(),
                                                                        frame->GetFilename(),
                                                                        frame->GetSensorConstants(),
                                                                        frame->Id(),
                                                                        frame->GetFeatureHandler()),
                                                               BaseMonocular(*frame) {
  SetStagingPosition(frame->GetPosition());
  ApplyStaging();
  SetMap(frame->GetMap());

}

void MonocularKeyFrame::InitializeImpl() {
  auto mp = map_points_.begin();

  while (mp!= map_points_.end()) {
    if(mp->second->IsBad()){
      mp = map_points_.erase(mp);
      continue;
    }
    mp->second->AddObservation(Observation(mp->second, this, mp->first));
    mp->second->ComputeDistinctiveDescriptor(GetFeatureHandler()->GetFeatureExtractor());
    mp->second->CalculateNormalStaging();
    mp->second->ApplyNormalStaging();
    ++mp;
  }
  logging::RetrieveLogger()->debug("Created new keyframe with id {}", Id());
  logging::RetrieveLogger()->debug("Number of map points:  {}", map_points_.size());

  covisibility_graph_.Update();
}

FrameType MonocularKeyFrame::Type() const {
  return MONOCULAR;
}

void MonocularKeyFrame::ListMapPoints(BaseFrame::MapPointSet & out_map_points) const {
  BaseMonocular::ListMapPoints(out_map_points);
}

TVector3D MonocularKeyFrame::GetNormal(const TPoint3D & point) const {
  TPoint3D normal = GetInversePosition().T - point;
  normal.normalize();
  return normal;
}

TVector3D MonocularKeyFrame::GetNormalFromStaging(const TPoint3D & point) const {
  TPoint3D normal = GetStagingPosition().GetInversePose().T - point;
  normal.normalize();
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

  for (auto mp:map_points) {
    if (mp->IsBad())
      continue;
    depths.push_back(R_z.dot(mp->GetPosition()) + pose.T.z());
  }

  return depths[(depths.size() - 1) / q];
}

void MonocularKeyFrame::CreateNewMapPoints(frame::KeyFrame * other, MapPointSet & out_newly_created) {

  auto other_frame = dynamic_cast<MonocularKeyFrame *> (other);
  if (other_frame == this)
    return;

  MonocularMapPoints local_map_points_map = GetMapPoints(), others_map_points_map = other_frame->GetMapPoints();

  MapPointSet local_map_points, others_map_points;
  std::transform(local_map_points_map.begin(),
                 local_map_points_map.end(),
                 std::inserter(local_map_points, local_map_points.begin()),
                 [](const std::pair<size_t, map::MapPoint *> & mp_id) { return mp_id.second; });

  std::transform(others_map_points_map.begin(),
                 others_map_points_map.end(),
                 std::inserter(others_map_points, others_map_points.begin()),
                 [](const std::pair<size_t, map::MapPoint *> & mp_id) { return mp_id.second; });

  auto local_pose = GetPositionWithLock();
  auto other_pose = other_frame->GetPositionWithLock();
  if (!BaseLineIsEnough(others_map_points, local_pose, other_pose)) {
    logging::RetrieveLogger()->debug("Baseline between frames {} and {} is not enough", Id(), other->Id());
    return;
  }

  logging::RetrieveLogger()->debug("LM: Initial local map point count: {}", local_map_points.size());

  if (other->Type() != Type()) {
    throw std::runtime_error("Matching Stereo With Monocular is not implemented yet");
  }

  features::FastMatches matches;
  feature_handler_->FastMatch(other_frame->GetFeatureHandler(), matches, features::MatchingSeverity::STRONG, true);

  geometry::Pose relative_pose;
  geometry::utils::ComputeRelativeTransformation(GetPosition(), other_frame->GetPosition(), relative_pose);

  logging::RetrieveLogger()->debug("LM: SNNMatcher found {} matches between {} and {}",
                                   matches.size(),
                                   other_frame->Id(),
                                   Id());

  unsigned newly_created_mps = 0;

  for (auto match: matches) {
    if (map_points_.find(match.first) != map_points_.end()
        || other_frame->map_points_.find(match.second) != other_frame->map_points_.end())
      continue;
    precision_t parallax;
    TPoint3D triangulated;
    if (!geometry::utils::TriangulateAndValidate(other_frame->feature_handler_->GetFeatures().undistorted_and_unprojected_keypoints[match.second],
                                                 feature_handler_->GetFeatures().undistorted_and_unprojected_keypoints[match.first],
                                                 relative_pose,
                                                 5.991 * GetCamera()->FxInv() * GetCamera()->FxInv(),
                                                 5.991 * other_frame->GetCamera()->FxInv()
                                                     * other_frame->GetCamera()->FxInv(),
                                                 constants::PARALLAX_THRESHOLD,
                                                 parallax,
                                                 triangulated))
      continue;
    TPoint2D pt_other, pt_this;
    other_frame->GetCamera()->ProjectAndDistort(triangulated, pt_other);
    GetCamera()->ProjectAndDistort(relative_pose.Transform(triangulated), pt_this);
    if (!other_frame->GetCamera()->IsInFrustum(pt_other))
      continue;
    if (!GetCamera()->IsInFrustum(pt_this))
      continue;
//    TPoint3D world_pos = other_frame->GetInversePosition().Transform(triangulated);
//    assert(world_pos.z() > 0);

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
//    std::cout << map_point->GetPosition() << std::endl;

    AddMapPoint(map_point, match.first);
    other_frame->AddMapPoint(map_point, match.second);

    map_point->ComputeDistinctiveDescriptor(feature_handler_->GetFeatureExtractor());
    map_point->CalculateNormalStaging();
    map_point->ApplyNormalStaging();
    out_newly_created.insert(map_point);
    ++newly_created_mps;
  }
  logging::RetrieveLogger()->debug("LM: Created {} new map_points between frames {} and {}",
                                   newly_created_mps,
                                   other_frame->Id(),
                                   Id());

}

void MonocularKeyFrame::FuseMapPoints(MapPointSet & map_points, bool use_staging) {
  std::list<MapPointVisibilityParams> visibles;
  FilterVisibleMapPoints(map_points, visibles, use_staging);
  typedef features::matching::iterators::ProjectionSearchIterator IteratorType;
  IteratorType begin(visibles.begin(), visibles.end(), &feature_handler_->GetFeatures());
  IteratorType end(visibles.end(), visibles.end(), &feature_handler_->GetFeatures());
  typedef features::matching::SNNMatcher<IteratorType> MatcherType;
  MatcherType::MatchMapType matches;
  MatcherType matcher(1., 50);
  matcher.MatchWithIterators(begin, end, feature_handler_->GetFeatureExtractor(), matches);

  MonocularMapPoints local_map_points = GetMapPoints();

  for (auto match: matches) {
    auto it = local_map_points.find(match.second);
    if (it == local_map_points.end()) {
      if (match.first->IsInKeyFrame(this))
        continue;
      AddMapPoint(match.first, match.second);
    } else {
      if (it->second == match.first)
        continue;;
      if (match.first->GetObservationCount() > it->second->GetObservationCount()) {
        it->second->SetReplaced(match.first);
        this->map_points_[match.second] = match.first;
      } else {
        match.first->SetReplaced(it->second);
      }
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
                                               bool use_staging) {
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
                                      GetFeatureExtractor()))
        out_visibles.push_back(visible_map_point);
    }
  }
}

void MonocularKeyFrame::AddMapPoint(map::MapPoint * map_point, size_t feature_id) {
  assert(!map_point->IsBad());
  BaseMonocular::AddMapPoint(map_point, feature_id);
  map_point->AddObservation(Observation(map_point, this, feature_id));
}

void MonocularKeyFrame::EraseMapPointImpl(const map::MapPoint * map_point, bool check_bad) {
  assert(nullptr != map_point);
  map::MapPoint::MapType observations = map_point->Observations();
  auto f = observations.find(this);
  assert(f != observations.end());
  EraseMapPointImpl(f->second.GetFeatureId(), check_bad);
}

void MonocularKeyFrame::EraseMapPointImpl(size_t feature_id, bool check_bad) {
  auto m_it = this->map_points_.find(feature_id);
  assert(m_it != this->map_points_.end());
  m_it->second->EraseObservation(this);
  if (check_bad && m_it->second->GetObservationCount() == 1) {
    m_it->second->SetBad();
  }
  BaseMonocular::EraseMapPoint(feature_id);
}

void MonocularKeyFrame::EraseMapPoint(const map::MapPoint * map_point) {
  EraseMapPointImpl(map_point, true);
}

void MonocularKeyFrame::EraseMapPoint(size_t feature_id) {
  EraseMapPointImpl(feature_id, true);
}

void MonocularKeyFrame::ReplaceMapPoint(map::MapPoint * map_point, const Observation & observation) {

  assert(observation.GetKeyFrame() == this);
  assert(map_points_.find(observation.GetFeatureId()) != map_points_.end());

  EraseMapPointImpl(observation.GetMapPoint(), false);
  if (map_point->IsInKeyFrame(this)) {
    EraseMapPointImpl(map_point, false);
  }
  AddMapPoint(map_point, observation.GetFeatureId());
}

void MonocularKeyFrame::SetBad() {
  while (!map_points_.empty()) {
    auto mp_it = map_points_.begin();
    assert(!mp_it->second->IsBad());
    EraseMapPoint(mp_it->second);
  }
  KeyFrame::SetBad();
}

void MonocularKeyFrame::SerializeToStream(std::ostream & stream) const {
  WRITE_TO_STREAM(id_, stream);
  WRITE_TO_STREAM(bad_flag_, stream);
  BaseMonocular::SerializeToStream(stream);
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

    auto local_mp_it = map_points_.find(match.first);
    if (local_mp_it == map_points_.end()) continue;
    map::MapPoint * local_map_point = local_mp_it->second;

    auto kf_map_point_it = mono_other->map_points_.find(match.second);
    if (kf_map_point_it == mono_other->map_points_.end()) continue;
    map::MapPoint * kf_map_point = kf_map_point_it->second;
    if (!local_map_point->IsBad() && !kf_map_point->IsBad())
      out_matches.emplace_back(local_map_point, kf_map_point);
  }

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

  const camera::MonocularCamera * local_camera = GetCamera();
  const camera::MonocularCamera
      * loop_candidate_camera = mono_loop_cand->GetCamera();

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

void MonocularKeyFrame::FilterVisibleMapPoints(const MapPointSet & map_points,
                                               const geometry::Sim3Transformation & relative_transformation,
                                               const geometry::Pose & mp_local_transformation,
                                               std::list<MapPointVisibilityParams> & out_visibles,
                                               precision_t radius_multiplier) const {
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
                                    GetFeatureExtractor()));
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
  matcher.MatchWithIterators(begin, end, GetFeatureExtractor(), matches);
  std::cout << "Match count: " << matches.size() << std::endl;
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

}
}
}
