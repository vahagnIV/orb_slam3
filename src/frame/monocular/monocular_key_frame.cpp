//
// Created by vahagn on 11/05/2021.
//

#include "monocular_key_frame.h"
#include "monocular_frame.h"
#include <map/map_point.h>
#include <logging.h>
#include <features/matching/iterators/bow_to_iterator.h>
#include <features/matching/iterators/projection_search_iterator.h>
#include <features/matching/second_nearest_neighbor_matcher.hpp>
#include <features/matching/validators/triangulation_validator.h>
#include <geometry/utils.h>

namespace orb_slam3 {
namespace frame {
namespace monocular {

MonocularKeyFrame::MonocularKeyFrame(MonocularFrame * frame) : KeyFrame(frame->GetTimeCreated(),
                                                                        frame->GetFilename(),
                                                                        frame->GetFeatureExtractor(),
                                                                        frame->GetVocabulary(),
                                                                        frame->GetSensorConstants(),
                                                                        frame->Id()),
                                                               BaseMonocular(*frame) {
  SetPosition(frame->GetPosition());
  for (auto mp: map_points_) {
    mp.second->AddObservation(Observation(mp.second, this, mp.first));
    mp.second->Refresh(feature_extractor_);
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

void MonocularKeyFrame::CreateNewMapPoints(frame::KeyFrame * other) {

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

  other_frame->ComputeBow();
  typedef features::matching::SNNMatcher<features::matching::iterators::BowToIterator> SNNM;
  SNNM bow_matcher(0.6, 50);
  features::matching::iterators::BowToIterator bow_it_begin(features_.bow_container.feature_vector.begin(),
                                                            &features_.bow_container.feature_vector,
                                                            &other_frame->features_.bow_container.feature_vector,
                                                            &features_,
                                                            &other_frame->features_,
                                                            &local_map_points_map,
                                                            &others_map_points_map,
                                                            false,
                                                            false);

  features::matching::iterators::BowToIterator bow_it_end(features_.bow_container.feature_vector.end(),
                                                          &features_.bow_container.feature_vector,
                                                          &other_frame->features_.bow_container.feature_vector,
                                                          &features_,
                                                          &other_frame->features_,
                                                          &local_map_points_map,
                                                          &others_map_points_map,
                                                          false,
                                                          false);
  geometry::Pose relative_pose;
  geometry::utils::ComputeRelativeTransformation(GetPosition(), other_frame->GetPosition(), relative_pose);

  features::matching::validators::TriangulationValidator
      validator(&features_, &other_frame->features_, &relative_pose, feature_extractor_, GetCamera()->FxInv());
  bow_matcher.AddValidator(&validator);

  SNNM::MatchMapType matches;
  bow_matcher.MatchWithIterators(bow_it_begin, bow_it_end, feature_extractor_, matches);

  logging::RetrieveLogger()->debug("LM: SNNMatcher found {} matches between {} and {}",
                                   matches.size(),
                                   other_frame->Id(),
                                   Id());

  unsigned newly_created_mps = 0;

  for (auto match: matches) {
    precision_t parallax;
    TPoint3D triangulated;
    if (!geometry::utils::TriangulateAndValidate(other_frame->features_.undistorted_and_unprojected_keypoints[match.second],
                                                 features_.undistorted_and_unprojected_keypoints[match.first],
                                                 relative_pose,
                                                 5.991 * GetCamera()->FxInv() * GetCamera()->FxInv(),
                                                 5.991 * other_frame->GetCamera()->FxInv()
                                                     * other_frame->GetCamera()->FxInv(),
                                                  constants::PARALLAX_THRESHOLD,
                                                 parallax,
                                                 triangulated))
      continue;

    precision_t min_invariance_distance, max_invariance_distance;
    feature_extractor_->ComputeInvariantDistances(triangulated,
                                                  other_frame->features_.keypoints[match.second],
                                                  max_invariance_distance,
                                                  min_invariance_distance);
    auto map_point = new map::MapPoint(other_frame->GetInversePosition().Transform(triangulated),
                                       Id(),
                                       max_invariance_distance,
                                       min_invariance_distance);
    std::cout << map_point->GetPosition() << std::endl;

    map_point->AddObservation(Observation(map_point, this, match.first));
    map_point->AddObservation(Observation(map_point, other, match.second));


    AddMapPoint(map_point, match.first);
    other_frame->AddMapPoint(map_point, match.second);
    map_point->Refresh(feature_extractor_);
    ++newly_created_mps;
  }
  logging::RetrieveLogger()->debug("LM: Created {} new map_points between frames {} and {}",
                                   newly_created_mps,
                                   other_frame->Id(),
                                   Id());

}

void MonocularKeyFrame::ComputeBow() {
  BaseMonocular::ComputeBow();
}

void MonocularKeyFrame::FuseMapPoints(BaseFrame::MapPointSet & map_points) {
  std::list<VisibleMapPoint> visibles;
  FilterVisibleMapPoints(map_points, visibles);
  typedef features::matching::iterators::ProjectionSearchIterator IteratorType;
  IteratorType begin(visibles.begin(), visibles.end(), &features_, nullptr);
  IteratorType end(visibles.end(), visibles.end(), &features_, nullptr);
  typedef features::matching::SNNMatcher<IteratorType> MatcherType;
  MatcherType::MatchMapType matches;
  MatcherType matcher(1., 50);
  matcher.MatchWithIterators(begin, end, feature_extractor_, matches);

  MonocularMapPoints local_map_points = GetMapPoints();

  for (auto match: matches) {
    auto it = local_map_points.find(match.second);
    if (it == local_map_points.end()) {
      match.first->AddObservation(Observation(match.first, this, match.second));
      AddMapPoint(match.first, match.second);
    } else {
      if (it->second == match.first)
        continue;;
      if (match.first->GetObservationCount() > it->second->GetObservationCount()) {
        it->second->SetReplaced(match.first);
        this->map_points_[match.second] = match.first;
      } else
        match.first->SetReplaced(it->second);
    }
  }
}

bool MonocularKeyFrame::IsVisible(map::MapPoint * map_point,
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

void MonocularKeyFrame::FilterVisibleMapPoints(const BaseFrame::MapPointSet & map_points,
                                               list<VisibleMapPoint> & out_visibles) {
  VisibleMapPoint visible_map_point;
  MapPointSet local_map_points;
  ListMapPoints(local_map_points);
  for (auto mp: map_points) {
    if (local_map_points.find(mp) == local_map_points.end()
        && IsVisible(mp, visible_map_point, GetSensorConstants()->max_allowed_discrepancy,0 ))
      out_visibles.push_back(visible_map_point);
  }
}

void MonocularKeyFrame::EraseMapPoint(Observation & obs) {
  map_points_.erase(obs.GetFeatutreIds()[0]);
}

}
}
}