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
                               TimePoint  time_point,
                               const string & filename,
                               const features::IFeatureExtractor * feature_extractor,
                               const camera::MonocularCamera * camera,
                               const features::BowVocabulary * const vocabulary) : Frame(time_point,
                                                                                         filename,
                                                                                         feature_extractor,
                                                                                         vocabulary),
                                                                                   BaseMonocular(image.cols(),
                                                                                                 image.rows(),
                                                                                                 camera) {

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
/*
#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << pose_.R << std::endl << pose_.T << std::endl;

    logging::RetrieveLogger()->debug(ss.str());

    std::ofstream ofstream("map_points_before_ba.bin", std::ios::binary | std::ios::out);

//

    for (const auto & mp: map_points) {

      orb_slam3::TPoint3D pose = mp->GetPosition();
      assert(!std::isnan(pose.x()) && !std::isnan(pose.y()) && !std::isnan(pose.z()));
//      std::cout << pose.x() << "\t" << pose.y() << "\t" << pose.z() << std::endl;
      ofstream.write(reinterpret_cast<char *>(&pose[0]), sizeof(decltype(pose[0])));
      ofstream.write(reinterpret_cast<char *>(&pose[1]), sizeof(decltype(pose[0])));
      ofstream.write(reinterpret_cast<char *>(&pose[2]), sizeof(decltype(pose[0])));
      orb_slam3::TVector3D normal = mp->GetNormal();
      ofstream.write(reinterpret_cast<char *>(&normal[0]), sizeof(decltype(normal[0])));
      ofstream.write(reinterpret_cast<char *>(&normal[1]), sizeof(decltype(normal[0])));
      ofstream.write(reinterpret_cast<char *>(&normal[2]), sizeof(decltype(normal[0])));
    }
  }

#endif

  std::unordered_set<Frame *> fixed{from_frame};
  g2o::SparseOptimizer optimizer;
  optimization::InitializeOptimizer(optimizer);
  optimizer.setVerbose(true);
  optimization::BundleAdjustment(optimizer,
                                 fixed,
                                 map_points,
                                 100);

  this->recent_observations_ = decltype(recent_observations_)(new std::unordered_set<Observation *>);
  from_frame->recent_observations_ = decltype(recent_observations_)(new std::unordered_set<Observation *>);

  for (auto map_point: map_points) {
    auto mp_vertex_base = optimizer.vertex(map_point->Id());
    auto mp_vertex = dynamic_cast<optimization::vertices::MapPointVertex *>(mp_vertex_base);
    bool all_edges_pass_threshold = true;
    for (auto edge_base: mp_vertex->edges()) {
      auto edge = dynamic_cast<optimization::edges::SE3ProjectXYZPose *>(edge_base);
      if (edge->chi2() > constants::MONO_CHI2 * camera_->FxInv() * camera_->FxInv()) {
        all_edges_pass_threshold = false;
        break;
      }
    }

    for (auto observation: map_point->Observations()) {
      if (all_edges_pass_threshold) {
        dynamic_cast<MonocularFrame *>(observation.first)->recent_observations_->insert(observation.second);
        observation.second->GetMapPoint()->IncreaseVisible();
        observation.second->GetMapPoint()->IncreaseFound();
      } else {
        observation.second->GetMapPoint()->EraseObservation(observation.first);
        delete observation.second;
      }
    }
    if (all_edges_pass_threshold) {
      map_point->SetPosition(mp_vertex->estimate());
      map_point->Refresh(feature_extractor_);
    } else {
      assert(map_point->Observations().empty());
      delete map_point;
    }
  }

  SetPosition(dynamic_cast<optimization::vertices::FrameVertex *>(optimizer.vertex(Id()))->estimate());
  logging::RetrieveLogger()->debug("Total map point count: {}", map_points_.size());*/

#ifndef NDEBUG
  {
    std::stringstream ss;

    ss << GetPosition().R << std::endl << GetPosition().T << std::endl;
    logging::RetrieveLogger()->debug("LINKING: Frame {} position after BA:", Id());
    logging::RetrieveLogger()->debug(ss.str());

//    from_frame->SetPosition(dynamic_cast<optimization::vertices::FrameVertex *>(optimizer.vertex(from_frame->Id()))->estimate());
//
//    std::ofstream ofstream("map_points_after_ba.bin", std::ios::binary | std::ios::out);
//
//    for (const auto & mp: map_points_) {
//      orb_slam3::TPoint3D pose = mp.second->GetPosition();
//      ofstream.write(reinterpret_cast<char *>(&pose[0]), sizeof(decltype(pose[0])));
//      ofstream.write(reinterpret_cast<char *>(&pose[1]), sizeof(decltype(pose[0])));
//      ofstream.write(reinterpret_cast<char *>(&pose[2]), sizeof(decltype(pose[0])));
//      orb_slam3::TVector3D normal = mp.second->GetNormal();
//      ofstream.write(reinterpret_cast<char *>(&normal[0]), sizeof(decltype(normal[0])));
//      ofstream.write(reinterpret_cast<char *>(&normal[1]), sizeof(decltype(normal[0])));
//      ofstream.write(reinterpret_cast<char *>(&normal[2]), sizeof(decltype(normal[0])));
//    }

    /*for (auto mp: map_points_) {
      std::map<MonocularFrame *, cv::Mat>
          images = {{this, cv::imread(Filename())}, {from_frame, cv::imread(from_frame->Filename())}};

      for (auto obs: mp.second->Observations()) {
        auto observation = dynamic_cast<MonocularObservation *>(obs.second);
        auto frame = dynamic_cast<MonocularFrame *> (observation->GetFrame());
        auto kp = frame->features_.keypoints[observation->GetFeatureId()];
        cv::circle(images[frame], cv::Point2f(kp.X(), kp.Y()), 3, cv::Scalar(0, 255, 0));
        auto point = frame->GetPose()->Transform(mp.second->GetPosition());
        TPoint2D projetcted;
        frame->GetCamera()->ProjectAndDistort(point, projetcted);
        cv::circle(images[frame], cv::Point2f(projetcted.x(), projetcted.y()), 3, cv::Scalar(0, 255, 255));
        cv::imshow(frame->filename_, images[frame]);
      }

      cv::waitKey();

    }*/
//    cv::waitKey();
  }
#endif
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
  features::matching::iterators::AreaToIterator end(GetFeatures().Size(), &GetFeatures(), &from_frame->GetFeatures(), 100);

  matcher.MatchWithIteratorV2(begin, end, feature_extractor_, out_matches);
  logging::RetrieveLogger()->debug("Orientation validator discarderd {} matches",
                                   features::matching::OrientationValidator
                                       (GetFeatures().keypoints, from_frame->GetFeatures().keypoints).Validate(out_matches));

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
    auto map_point = new map::MapPoint(point.second, max_invariance_distance, min_invariance_distance);
    AddMapPoint(map_point, point.first);
    from_frame->AddMapPoint(map_point, matches.find(point.first)->second);
    out_map_points.insert(map_point);
  }

}

void MonocularFrame::AddMapPoint(map::MapPoint * map_point, size_t feature_id) {
  assert(!MapPointExists(map_point));
  assert(map_points_.find(feature_id) == map_points_.end());
  map_points_[feature_id] = map_point;
}

bool MonocularFrame::MapPointExists(const map::MapPoint * map_point) const {
  for (auto mp: map_points_)
    if (mp.second == map_point)
      return true;
  return false;
}

bool MonocularFrame::IsValid() const {
  return GetFeatures().Size() > 0;
}

}
}
}