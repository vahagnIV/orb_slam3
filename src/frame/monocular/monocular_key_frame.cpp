//
// Created by vahagn on 11/05/2021.
//

#include "monocular_key_frame.h"
#include "monocular_frame.h"
#include <map/map_point.h>
#include <logging.h>
#include <features/handlers/DBoW2/bow_to_iterator.h>
#include <features/matching/iterators/projection_search_iterator.h>
#include <features/matching/second_nearest_neighbor_matcher.hpp>
#include <features/matching/validators/triangulation_validator.h>
#include <geometry/utils.h>
#include <geometry/ransac_sim3_solver.h>

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
  SetPosition(frame->GetPosition());
  SetMap(frame->GetMap());
  for (auto mp: map_points_) {
    mp.second->AddObservation(Observation(mp.second, this, mp.first));
    mp.second->Refresh(GetFeatureHandler()->GetFeatureExtractor());
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

precision_t MonocularKeyFrame::GetSimilarityScore(const BaseFrame * other) const {
  /* if (other->Type() != MONOCULAR) {
     return 0;
   }
   return vocabulary_->score(this->GetFeatures().bow_container.bow_vector,
                             dynamic_cast<const BaseMonocular *>(other)->GetFeatures().bow_container.bow_vector);*/
  return 0;
}

TVector3D MonocularKeyFrame::GetNormal(const TPoint3D & point) const {
  TPoint3D normal = GetInversePosition().T - point;
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

    map_point->Refresh(feature_handler_->GetFeatureExtractor());
    out_newly_created.insert(map_point);
    ++newly_created_mps;
  }
  logging::RetrieveLogger()->debug("LM: Created {} new map_points between frames {} and {}",
                                   newly_created_mps,
                                   other_frame->Id(),
                                   Id());

}

void MonocularKeyFrame::FuseMapPoints(BaseFrame::MapPointSet & map_points) {
  std::list<MapPointVisibilityParams> visibles;
  FilterVisibleMapPoints(map_points, visibles);
  typedef features::matching::iterators::ProjectionSearchIterator IteratorType;
  IteratorType begin(visibles.begin(), visibles.end(), &feature_handler_->GetFeatures(), nullptr);
  IteratorType end(visibles.end(), visibles.end(), &feature_handler_->GetFeatures(), nullptr);
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

bool MonocularKeyFrame::IsVisible(map::MapPoint * map_point,
                                  MapPointVisibilityParams & out_map_point,
                                  precision_t radius_multiplier,
                                  unsigned int window_size) const {
  return BaseMonocular::IsVisible(map_point,
                                  out_map_point,
                                  radius_multiplier,
                                  window_size,
                                  -1,
                                  this->GetPosition(),
                                  this->GetInversePosition(),
                                  feature_handler_->GetFeatureExtractor());
}

void MonocularKeyFrame::FilterVisibleMapPoints(const BaseFrame::MapPointSet & map_points,
                                               std::list<MapPointVisibilityParams> & out_visibles) {
  MapPointVisibilityParams visible_map_point;
  MapPointSet local_map_points;
  ListMapPoints(local_map_points);
  for (auto mp: map_points) {
    if (mp->IsBad()) continue;
    if (!mp->IsInKeyFrame(this)) {
      if (local_map_points.find(mp) == local_map_points.end()
          && IsVisible(mp, visible_map_point, GetSensorConstants()->max_allowed_discrepancy, 0))
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
  feature_handler_->FastMatch(dynamic_cast<const MonocularKeyFrame *>(other)->GetFeatureHandler(),
                              matches,
                              features::MatchingSeverity::WEAK,
                              true);

  for (const auto & match: matches) {
    map::MapPoint * local_map_point = map_points_.find(match.first)->second;
    map::MapPoint * kf_map_point =
        dynamic_cast<const MonocularKeyFrame *>(other)->map_points_.find(match.second)->second;
    if (local_map_point && !local_map_point->IsBad()
        && kf_map_point && !kf_map_point->IsBad())
      out_matches.emplace_back(local_map_point, kf_map_point);
  }

}

bool MonocularKeyFrame::FindSim3Transformation(const KeyFrame::MapPointMatches & map_point_matches,
                                               const KeyFrame * loop_candidate,
                                               geometry::Sim3Transformation & out_transormation) const {
  std::vector<std::pair<TPoint3D, TPoint3D>> matched_points(map_point_matches.size());
  std::vector<std::pair<TPoint2D, TPoint2D>> matched_point_projections(map_point_matches.size());
  std::transform(map_point_matches.begin(),
                 map_point_matches.end(),
                 matched_points.begin(),
                 [](const std::pair<map::MapPoint *, map::MapPoint *> & pair) {
                   return std::pair<TPoint3D, TPoint3D>(pair.first->GetPosition(), pair.second->GetPosition());
                 });
  const geometry::Pose kf_pose = GetPosition();
  const camera::MonocularCamera * local_camera = GetCamera();
  const camera::MonocularCamera
      * loop_candidate_camera = dynamic_cast<const MonocularKeyFrame *>(loop_candidate)->GetCamera();

  const geometry::Pose loop_candidate_pose = loop_candidate->GetPosition();
  std::transform(matched_points.begin(),
                 matched_points.end(),
                 matched_point_projections.begin(),
                 [&kf_pose, & loop_candidate_pose, & local_camera, &loop_candidate_camera]
                     (const std::pair<TPoint3D, TPoint3D> & match) {
                   TPoint2D local_projection, loop_candidate_projection;
                   local_camera->ProjectAndDistort(kf_pose.Transform(match.first), local_projection);
                   loop_candidate_camera->ProjectAndDistort(loop_candidate_pose.Transform(match.second),
                                                            loop_candidate_projection);
                   return std::pair<TPoint2D, TPoint2D>(local_projection, loop_candidate_projection);
                 });

  std::vector<std::pair<precision_t, precision_t>> errors;
  geometry::RANSACSim3Solver
      solver(&matched_points, &matched_point_projections, local_camera, loop_candidate_camera, &errors, 300);
  return solver(out_transormation);
}

}
}
}
