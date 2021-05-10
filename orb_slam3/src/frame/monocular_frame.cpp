//
// Created by vahagn on 1/23/21.
//

/// === Standard ===
#include <cassert>

// === g2o ===
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>

// == orb-slam3 ===
#include <frame/monocular_frame.h>
#include <frame/monocular_observation.h>
#include <constants.h>
#include <features/matching/second_nearest_neighbor_matcher.h>
#include <features/matching/iterators/bow_to_iterator.h>
#include <features/matching/orientation_validator.h>
#include <features/matching/validators/triangulation_validator.h>
#include <geometry/two_view_reconstructor.h>
#include <geometry/utils.h>
#include <optimization/edges/se3_project_xyz_pose.h>
#include <optimization/edges/se3_project_xyz_pose_only.h>
#include <logging.h>

#include <features/matching/iterators/area_to_iterator.h>
#include <features/matching/iterators/projection_search_iterator.h>

#include <debug/debug_utils.h>
#include <optimization/bundle_adjustment.h>

namespace orb_slam3 {
namespace frame {

MonocularFrame::MonocularFrame(const TImageGray8U & image, TimePoint timestamp,
                               const std::shared_ptr<features::IFeatureExtractor> & feature_extractor,
                               const std::string & filename,
                               const std::shared_ptr<camera::MonocularCamera> & camera,
                               features::BowVocabulary * vocabulary) :
    FrameBase(timestamp, feature_extractor, filename),
    features_(camera->Width(), camera->Height()),
    camera_(camera) {

  feature_extractor->Extract(image, features_);
  features_.UndistortKeyPoints(camera_);
  features_.AssignFeaturesToGrid();
  features_.SetVocabulary(vocabulary);
}

bool MonocularFrame::IsValid() const {
  return features_.descriptors.size() > constants::MINIMAL_FEATURE_COUNT_PER_FRAME_MONOCULAR;
}

bool MonocularFrame::ComputeMatchesForLinking(MonocularFrame * from_frame,
                                              unordered_map<size_t, size_t> & out_matches) {
  features::matching::SNNMatcher<features::matching::iterators::AreaToIterator> matcher(0.9, 50);
  features::matching::iterators::AreaToIterator begin(0, &features_, &from_frame->features_, 100);
  features::matching::iterators::AreaToIterator end(features_.Size(), &features_, &from_frame->features_, 100);

  matcher.MatchWithIteratorV2(begin, end, feature_extractor_.get(), out_matches);
  logging::RetrieveLogger()->debug("Orientation validator discarderd {} matches",
                                   features::matching::OrientationValidator
                                       (features_.keypoints, from_frame->features_.keypoints).Validate(out_matches));

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

void MonocularFrame::InitializeMapPointsFromMatches(const unordered_map<std::size_t, std::size_t> & matches,
                                                    const std::unordered_map<size_t, TPoint3D> & points,
                                                    MonocularFrame * from_frame,
                                                    unordered_set<map::MapPoint *> & out_map_points) {
  for (const auto & point : points) {

    precision_t max_invariance_distance, min_invariance_distance;
    feature_extractor_->ComputeInvariantDistances(pose_.Transform(point.second),
                                                  features_.keypoints[point.first],
                                                  max_invariance_distance,
                                                  min_invariance_distance);
    auto map_point = new map::MapPoint(point.second, max_invariance_distance, min_invariance_distance);

    auto this_observation = new MonocularObservation(map_point, this, point.first);
    auto from_observation = new MonocularObservation(map_point, from_frame, matches.find(point.first)->second);
    map_point->AddObservation(this_observation);
    map_point->AddObservation(from_observation);
    out_map_points.insert(map_point);
  }
}

bool MonocularFrame::Link(FrameBase * other) {

  logging::RetrieveLogger()->info("Linking frame {} with {}", Id(), other->Id());
  if (other->Type() != Type()) {
    throw std::runtime_error("Linking frames of different types is not implemented yet.");
  }

  MonocularFrame * from_frame = dynamic_cast<MonocularFrame *>(other);
  std::unordered_map<std::size_t, std::size_t> matches;
  if (!ComputeMatchesForLinking(from_frame, matches))
    return false;

  geometry::TwoViewReconstructor reconstructor(200, camera_->FxInv());
  std::unordered_map<size_t, TPoint3D> points;
  if (!reconstructor.Reconstruct(features_.undistorted_and_unprojected_keypoints,
                                 from_frame->features_.undistorted_and_unprojected_keypoints,
                                 matches,
                                 pose_,
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

  std::unordered_set<map::MapPoint *> map_points;
  InitializeMapPointsFromMatches(matches, points, from_frame, map_points);
  std::cout << " Mps: " << map_points.size() << std::endl;

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

  std::unordered_set<FrameBase *> fixed{from_frame};
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
  logging::RetrieveLogger()->debug("Total map point count: {}", map_points_.size());

#ifndef NDEBUG
  {
    std::stringstream ss;

    ss << pose_.R << std::endl << pose_.T << std::endl;
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
    cv::waitKey();
  }
#endif
  return true;
}

TVector3D MonocularFrame::GetNormal(const TPoint3D & point) const {
  TPoint3D normal = inverse_pose_.T - point;
  normal.normalize();
  return normal;
}

bool MonocularFrame::TrackWithReferenceKeyFrame(FrameBase * reference_keyframe) {

  logging::RetrieveLogger()->info("TWRKF: Tracking frame {} with reference keyframe {}",
                                  Id(),
                                  reference_keyframe->Id());
  if (reference_keyframe->Type() != Type()) {
    logging::RetrieveLogger()->warn("Frames {} and {} have different types", Id(), reference_keyframe->Id());
    return false;
  }
  auto reference_kf = dynamic_cast<MonocularFrame *>(reference_keyframe);

  // Ensure bows are computed
  reference_kf->ComputeBow();
  ComputeBow();

  std::unordered_map<std::size_t, std::size_t> matches;
  ComputeMatches(reference_kf, matches, false, true);

  logging::RetrieveLogger()->info("TWRKF: SNNMatcher returned {} matches for frames {} and {}",
                                  matches.size(),
                                  Id(),
                                  reference_keyframe->Id());
  if (matches.size() < 20) {
    return false;
  }

  cv::imshow("TrackWithReferenceKeyframe",
             debug::DrawMatches(Filename(), reference_kf->Filename(), matches, features_, reference_kf->GetFeatures()));
  cv::waitKey(1);

  // Add the existing map_point to the frame
  for (const auto & match: matches) {
    auto map_point = reference_kf->map_points_[match.second];
    recent_observations_->insert(new MonocularObservation(map_point, this, match.second));
  }
//  std::unordered_set<std::size_t> inliers;
//#ifndef NDEBUG
//  {
//    std::stringstream ss;
//    ss << "TWRKF: Tracking optimization. Pose before optimization\n";
//    ss << pose_.R << std::endl << pose_.T << std::endl;
//    logging::RetrieveLogger()->debug(ss.str());
//  }
//#endif
//
//  OptimizePose(inliers);
//
//  if (inliers.size() <= 4) {
//    logging::RetrieveLogger()->debug("TWRKF: not enough inliers after optimization");
//    return false;
//  }
//#ifndef NDEBUG
//  {
//    std::stringstream ss;
//    ss << "TWRKF: Tracking optimization. Pose after optimization\n";
//    ss << pose_.R << std::endl << pose_.T << std::endl;
//    logging::RetrieveLogger()->debug(ss.str());
//  }
//#endif
//  for (const auto & match: matches) {
//    if (inliers.find(match.first) == inliers.end()) {
//      EraseMapPoint(match.first);
//    }
//  }
  return true;
}

void MonocularFrame::ComputeBow() {
  features_.ComputeBow();
}

void MonocularFrame::OptimizePose(std::unordered_set<std::size_t> & out_inliers) {
/*
  static const precision_t delta_mono = constants::HUBER_MONO_DELTA * camera_->FxInv();
  static const precision_t chi2_threshold = constants::MONO_CHI2 * camera_->FxInv() * camera_->FxInv();

  g2o::SparseOptimizer optimizer;
  InitializeOptimizer(optimizer);

  g2o::VertexSE3Expmap * pose = CreatePoseVertex();
  optimizer.addVertex(pose);
  size_t last_id = Identifiable::GetNextId();
  std::unordered_map<optimization::edges::SE3ProjectXYZPoseOnly *, std::size_t> edges;

  for (auto & mp_id: map_points_) {
    map::MapPoint * map_point = mp_id.second;
    size_t feature_id = mp_id.first;
    if (nullptr == map_point)
      continue;
    out_inliers.insert(feature_id);
    auto edge = new optimization::edges::SE3ProjectXYZPoseOnly(camera_.get(), map_point->GetPosition());
    edge->setVertex(0, pose);
    precision_t
        information_coefficient = feature_extractor_->GetAcceptableSquareError(features_.keypoints[feature_id].level);
    edge->setInformation(
        Eigen::Matrix<double, 2, 2>::Identity() * information_coefficient);
    edge->setId(++last_id);
    auto rk = new g2o::RobustKernelHuber;
    rk->setDelta(delta_mono);
    edge->setLevel(0);
    edge->setRobustKernel(rk);
    HomogenousPoint measurement;
    edge->setMeasurement(Eigen::Map<Eigen::Matrix<double, 2, 1>>(features_.unprojected_keypoints[feature_id].data()));
    optimizer.addEdge(edge);
    edges[edge] = feature_id;
  }

  for (int i = 0; i < 4 && !out_inliers.empty(); ++i) {
    pose->setEstimate(pose_.GetQuaternion());
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    for (auto edge: edges) {
      if (out_inliers.find(edge.second) == out_inliers.end()) { // If  the edge was not included in the optimization
        edge.first->computeError();
      }
      if (edge.first->chi2() < chi2_threshold) {
        out_inliers.insert(edge.second);
        edge.first->setLevel(0);
      } else {
        out_inliers.erase(edge.second);
        edge.first->setLevel(1);
      }

      if (i == 2)
        edge.first->setRobustKernel(nullptr);
    }
  }

  SetPosition(pose->estimate());*/
}

precision_t MonocularFrame::ComputeMedianDepth() const {
  const g2o::SE3Quat pose_quat(pose_.R, pose_.T);
  std::vector<precision_t> depths(map_points_.size());
  std::transform(map_points_.begin(),
                 map_points_.end(),
                 depths.begin(),
                 [&pose_quat](const std::pair<size_t, map::MapPoint *> & map_point) -> precision_t {
                   return pose_quat.map(map_point.second->GetPosition()).z();
                 });
  return depths[(depths.size() - 1) / 2];
}

bool MonocularFrame::BaselineIsNotEnough(const MonocularFrame * other) const {
  TVector3D baseline = pose_.T - other->pose_.T;
  precision_t baseline_length = baseline.norm();
  precision_t frame_median_depth = other->ComputeMedianDepth();
  return baseline_length / frame_median_depth < 1e-2;
}

void MonocularFrame::ComputeMatches(MonocularFrame * reference_kf,
                                    std::unordered_map<std::size_t, std::size_t> & out_matches,
                                    bool self_keypoint_exists,
                                    bool reference_kf_keypoint_exists) {
/*  features::matching::SNNMatcher<features::matching::iterators::BowToIterator> bow_matcher(0.7, 50);
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

  bow_matcher.MatchWithIteratorV2(bow_it_begin, bow_it_end, feature_extractor_.get(), out_matches);
  features::matching::OrientationValidator
      (features_.keypoints, reference_kf->features_.keypoints).Validate(out_matches);*/

}

/*
void MonocularFrame::FindNewMapPointMatches(MonocularFrame * keyframe,
                                            std::unordered_map<std::size_t, std::size_t> & out_matches) {

  ComputeBow();
  keyframe->ComputeBow();

  if (BaselineIsNotEnough(keyframe)) {
    logging::RetrieveLogger()->debug("Baseline between frames  {} and {} is not enough", Id(), keyframe->Id());
    return;
  }
  ComputeMatches(keyframe, out_matches, false, false);

//  std::stringstream stringstream1;
//  stringstream1 << "/data/tmp/test-match/";
//  stringstream1 << Id() << "-" << keyframe->Id() << ".jpg";
//  cv::imwrite(stringstream1.str(),
//              debug::DrawMatches(Filename(), keyframe->Filename(), out_matches, features_, keyframe->features_));
  logging::RetrieveLogger()->debug("Local mapper: SNN Matcher found {} new matches between {} and {}",
                                   out_matches.size(),
                                   Id(),
                                   keyframe->Id());

}
*/
void MonocularFrame::CreateNewMpPoints(MonocularFrame * keyframe,
                                       const std::unordered_map<std::size_t, std::size_t> & matches,
                                       std::unordered_set<map::MapPoint *> & out_map_points) {
  /*for (auto & match: matches) {
    assert (map_points_.find(match.first) == map_points_.end());

    geometry::Pose relative_pose;
    geometry::utils::ComputeRelativeTransformation(pose_, keyframe->pose_, relative_pose);
    TPoint3D pt;
    precision_t parallax;
    if (!geometry::utils::TriangulateAndValidate(keyframe->features_.undistorted_and_unprojected_keypoints[match.second],
                                                 features_.undistorted_and_unprojected_keypoints[match.first],
                                                 relative_pose,
                                                 camera_->FxInv(),
                                                 keyframe->camera_->FxInv(),
                                                 constants::PARALLAX_THRESHOLD,
                                                 parallax,
                                                 pt))
      continue;

    precision_t max_invariance_distance, min_invariance_distance;
    feature_extractor_->ComputeInvariantDistances(relative_pose.Transform(pt),
                                                  features_.keypoints[match.first],
                                                  max_invariance_distance,
                                                  min_invariance_distance);
    auto map_point = new map::MapPoint(keyframe->GetInversePose()->Transform(pt),
                                       max_invariance_distance,
                                       min_invariance_distance);
//    AddMapPoint(map_point, match.first);
//    map_point->AddObservation(keyframe, match.second);
//    map_point->AddObservation(this, match.first);
    out_map_points.insert(map_point);

  }*/

}

void MonocularFrame::SearchLocalPoints(unordered_set<map::MapPoint *> & all_candidate_map_points) {
  /*std::unordered_set<map::MapPoint *> current_frame_map_points, all_map_points_except_locals;
  this->ListMapPoints(current_frame_map_points);
  SetDiff(all_candidate_map_points, current_frame_map_points, all_map_points_except_locals);
  std::unordered_map<map::MapPoint *, std::size_t> matches;
  std::list<VisibleMapPoint> candidate_map_points;
  FilterVisibleMapPoints(all_map_points_except_locals, candidate_map_points);
  FindCandidateMapPointMatchesByProjection(candidate_map_points, matches);
//  features::matching::SNNMatcher matcher(constants::NNRATIO_MONOCULAR_TWMM, constants::MONO_TWMM_THRESHOLD_HIGH);

  logging::RetrieveLogger()->debug("SLMP: Found {} matches for threshold 1.", matches.size());
  for (auto & match:matches) {
    AddMapPoint(match.first, match.second);
  }

  std::unordered_set<size_t> inliers;
  OptimizePose(inliers);
  for (auto mp_it = map_points_.begin(); mp_it != map_points_.end();) {
    if (inliers.find(mp_it->first) == inliers.end()) {
      mp_it = EraseMapPoint(mp_it);
    } else {
      if (all_map_points_except_locals.find(mp_it->second) != all_map_points_except_locals.end())
        mp_it->second->IncreaseFound();
//      mp_it->second->AddObservation(this, mp_it->first);
      mp_it->second->Refresh(feature_extractor_);
      ++mp_it;
    }
  }

#ifndef NDEBUG
  logging::RetrieveLogger()->debug("SLMP: {} MPs after pose optimization ", map_points_.size());
  std::stringstream ss;
  ss << "SLMP: Pose optimization. Pose after optimization\n";
  ss << pose_.R << std::endl << pose_.T << std::endl;
  logging::RetrieveLogger()->debug(ss.str());
  cv::Mat current_image = cv::imread(Filename(), cv::IMREAD_COLOR);
  for (auto mp: map_points_) {
    cv::circle(current_image,
               cv::Point(features_.keypoints[mp.first].X(), features_.keypoints[mp.first].Y()),
               3,
               cv::Scalar(0, 255, 0));
  }
  cv::imshow("SLMP", current_image);
  cv::waitKey(1);
#endif*/
}

/*
optimization::edges::SE3ProjectXYZPose * MonocularFrame::CreateEdge(map::MapPoint * map_point, MonocularFrame * frame) {
  // TODO: this assumes that the camera is the same for all frames
  const precision_t delta_mono = constants::HUBER_MONO_DELTA * frame->camera_->FxInv();
  auto edge = new optimization::edges::SE3ProjectXYZPose(frame->camera_.get());
  auto measurement = frame->features_.unprojected_keypoints[map_point->Observations()[frame]];
  edge->setMeasurement(Eigen::Map<Eigen::Matrix<double,
                                                2,
                                                1>>(measurement.data()));
  precision_t
      information_coefficient =
      frame->GetFeatureExtractor()->GetAcceptableSquareError(frame->features_.keypoints[map_point->Observations()[frame]].level);
  edge->setInformation(Eigen::Matrix2d::Identity() * information_coefficient);
  auto rk = new g2o::RobustKernelHuber;
  rk->setDelta(delta_mono);
  edge->setRobustKernel(rk);
  return edge;
}

bool MonocularFrame::FindNewMapPoints() {

  if (neighbour_keyframes == fixed_frames) {
    for (auto map_point: existing_local_map_points) {
      for (auto obs : map_point->Observations()) {
        if (dynamic_cast<MonocularFrame *>(obs.first)->map_points_.find(obs.second)
            == dynamic_cast<MonocularFrame *>(obs.first)->map_points_.end())
          dynamic_cast<MonocularFrame *>(obs.first)->AddMapPoint(map_point, obs.second);
      }
      map_point->Refresh(feature_extractor_);
    }
    logging::RetrieveLogger()->debug("FNMPs: map_point count: {}", existing_local_map_points.size());
    return true;
  }
  std::unordered_map<map::MapPoint *, std::unordered_set<MonocularFrame *>> inliers;
  BundleAdjustment(neighbour_keyframes, fixed_frames, existing_local_map_points, inliers, 5);

  unsigned new_deleted = 0, old_deleted = 0, erased_new_connection = 0, erased_old_connection = 0;
  for (auto map_point: existing_local_map_points) {
    auto observations = map_point->Observations();
    for (auto obs: observations) {
      auto frame = dynamic_cast<MonocularFrame *>(obs.first);
      if (neighbour_keyframes.find(frame) == neighbour_keyframes.end())
        continue;
      size_t feature_id = obs.second;
      if (inliers.find(map_point) != inliers.end() && inliers[map_point].find(frame) != inliers[map_point].end()) {
        auto existing_map_point = frame->map_points_.find(feature_id);
        if (existing_map_point == frame->map_points_.end()) {
          frame->AddMapPoint(map_point, feature_id);
        } else {
          if (existing_map_point->second == map_point)
            continue;
          TPoint2D exiting_projection, new_projection;
          frame->camera_->ProjectAndDistort(existing_map_point->second->GetPosition(), exiting_projection);
          frame->camera_->ProjectAndDistort(map_point->GetPosition(), new_projection);
          precision_t existing_distance = (frame->features_.keypoints[feature_id].pt - exiting_projection).norm();
          precision_t new_distance = (frame->features_.keypoints[feature_id].pt - new_projection).norm();
          if (map_point->Id() >= min_new_map_point_id)
            ++erased_new_connection;
          else
            ++erased_old_connection;
          if (existing_distance > new_distance) {
            frame->EraseMapPoint(feature_id);
            existing_map_point->second->EraseObservation(frame);
          } else
            map_point->EraseObservation(frame);
        }
      } else {
        frame->EraseMapPoint(feature_id);
        map_point->EraseObservation(frame);
      }
    }
  }

  for (auto map_point: existing_local_map_points) {
    if (map_point->Observations().size() <= 1) {
      for (auto frame: map_point->Observations()) {
        dynamic_cast<MonocularFrame *>(frame.first)->EraseMapPoint(frame.second);
      }
      if (map_point->Id() >= min_new_map_point_id)
        ++new_deleted;
      else
        ++old_deleted;
      delete map_point;
    } else
      map_point->Refresh(feature_extractor_);
  }
  logging::RetrieveLogger()->debug("Deleted {} new and {} old map points", new_deleted, old_deleted);
  logging::RetrieveLogger()->debug("Erased {} new and {} old connections",
                                   erased_new_connection,
                                   erased_old_connection);
  logging::RetrieveLogger()->debug("Total map points {}", map_points_.size());
  for (auto frame: neighbour_keyframes) {
    frame->CovisibilityGraph().Update();
  }

  std::stringstream ss;
  ss << "LM: Local bundle adjustment. Pose after optimization\n";
  ss << pose_.R << std::endl << pose_.T << std::endl;
  logging::RetrieveLogger()->debug(ss.str());

  cv::Mat current_image = cv::imread(Filename(), cv::IMREAD_COLOR);
  for (auto mp: map_points_) {
    cv::circle(current_image,
               cv::Point(features_.keypoints[mp.first].X(), features_.keypoints[mp.first].Y()),
               3,
               cv::Scalar(0, 255, 0));
  }
  cv::imshow("LocalMapper", current_image);
  cv::waitKey(1);

  return map_points_.size() > 10;
}
*/
bool MonocularFrame::IsVisible(map::MapPoint * map_point,
                               VisibleMapPoint & out_map_point,
                               precision_t radius_multiplier,
                               unsigned window_size) const {
  out_map_point.map_point = map_point;
  HomogenousPoint map_point_in_local_cf = pose_.Transform(map_point->GetPosition());
  precision_t distance = map_point_in_local_cf.norm();

  if (distance < map_point->GetMinInvarianceDistance()
      || distance > map_point->GetMaxInvarianceDistance()) {
    return false;
  }

  camera_->ProjectAndDistort(map_point_in_local_cf, out_map_point.position);
  if (!camera_->IsInFrustum(out_map_point.position)) {
    return false;
  }

  TPoint3D local_pose = inverse_pose_.T;
  TVector3D relative_frame_map_point = local_pose - map_point->GetPosition();

  precision_t track_view_cos = relative_frame_map_point.dot(map_point->GetNormal()) / relative_frame_map_point.norm();
  if (track_view_cos < 0.5) {
    return false;
  }

  out_map_point.level = feature_extractor_->PredictScale(distance, map_point->GetMaxInvarianceDistance() / 1.2);
  if (window_size) {
    out_map_point.window_size = window_size;
  } else {
    precision_t r = radius_multiplier * (track_view_cos > 0.998 ? 2.5 : 4.0);
    out_map_point.window_size = r * feature_extractor_->GetScaleFactors()[out_map_point.level];
  }

  return true;
}

void MonocularFrame::FilterVisibleMapPoints(const std::unordered_set<map::MapPoint *> map_points,
                                            list<VisibleMapPoint> & out_filetered_map_points,
                                            precision_t radius_multiplier,
                                            unsigned int window_size) const {
  VisibleMapPoint vmp;
  for (map::MapPoint * map_point: map_points) {
    if (IsVisible(map_point, vmp, radius_multiplier, window_size))
      out_filetered_map_points.push_back(vmp);
  }
}

void MonocularFrame::FindCandidateMapPointMatchesByProjection(const list<VisibleMapPoint> & filtered_map_points,
                                                              unordered_map<map::MapPoint *,
                                                                            std::size_t> & out_matches) {
  /*features::matching::iterators::ProjectionSearchIterator begin
      (filtered_map_points.begin(),
       filtered_map_points.end(),
       &features_,
       &map_points_);

  features::matching::iterators::ProjectionSearchIterator end
      (filtered_map_points.end(),
       filtered_map_points.end(),
       &features_,
       &map_points_);

  features::matching::SNNMatcher<features::matching::iterators::ProjectionSearchIterator>
      matcher(constants::NNRATIO_MONOCULAR_TWMM, constants::MONO_TWMM_THRESHOLD_HIGH);
  matcher.MatchWithIteratorV2(begin, end, feature_extractor_.get(), out_matches);
   */

}

bool MonocularFrame::FindNewMapPointsAndAdjustPosition(const std::unordered_set<map::MapPoint *> & all_candidate_map_points) {
  /*std::list<VisibleMapPoint> filtered_map_points;
  std::unordered_set<map::MapPoint *> current_map_points;
  ListMapPoints(current_map_points);
  std::unordered_set<map::MapPoint *> all_map_points_except_local;
  SetDiff(all_candidate_map_points, current_map_points, all_map_points_except_local);
  FilterVisibleMapPoints(all_map_points_except_local, filtered_map_points, 1, 15);
  std::unordered_map<map::MapPoint *, std::size_t> matches;
  FindCandidateMapPointMatchesByProjection(filtered_map_points, matches);
  logging::RetrieveLogger()->debug("TWMM: Found {} matches for threshold 15.", matches.size());

  if (matches.size() < 20) {
    matches.clear();
    for (auto & filtered: filtered_map_points)
      filtered.window_size = 30;
    FindCandidateMapPointMatchesByProjection(filtered_map_points, matches);
    logging::RetrieveLogger()->debug("TWMM: Found {} matches for threshold 30.", matches.size());
  }

  if (matches.size() < 20) {
    logging::RetrieveLogger()->debug("TWMM: Not enough map points for optimization");
    return false;
  }

  for (auto & filtered: filtered_map_points) {
    filtered.map_point->IncreaseVisible();
  }

#ifndef NDEBUG
  {
//    std::map<MonocularFrame *, std::unordered_map<std::size_t, size_t>> match_map;
//    for (auto match:matches) {
//      map::MapPoint * mp = match.first;
//      size_t current_feature_id = match.second;
//      for (auto obs: mp->Observations()) {
//        match_map[dynamic_cast<MonocularFrame *>(obs.first)][current_feature_id] = obs.second;
//      }
//    }
//    for (auto & m: match_map) {
//      std::stringstream ss;
//      ss << "tml-" << Id() << "-" << m.first->Id();
//      cv::imshow(ss.str(),
//                 debug::DrawMatches(Filename(), m.first->Filename(), m.second, features_, m.first->features_));
//
//    }
//    cv::waitKey();
  }
#endif

  for (auto match: matches) {
    AddMapPoint(match.first, match.second);
  }

#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << "TWMM: Tracking optimization. Pose before optimization\n";
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif

  std::unordered_set<std::size_t> inliers;
  OptimizePose(inliers);
  if (inliers.size() < 10) {
    map_points_.clear();
    logging::RetrieveLogger()->debug("TWMM: Not enough inliers after optimization");
    return false;
  }

#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << "TWMM: Tracking optimization. Pose after optimization\n";
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif

  for (decltype(map_points_)::iterator mp_it = map_points_.begin(); mp_it != map_points_.end();) {
    if (inliers.find(mp_it->first) == inliers.end())
      mp_it = EraseMapPoint(mp_it);
    else {
      if (all_map_points_except_local.find(mp_it->second) != all_map_points_except_local.end())
        mp_it->second->IncreaseFound();
      ++mp_it;
    }
  }

  logging::RetrieveLogger()->debug("TWMM: Map point count after optimization: {}", inliers.size());

#ifndef NDEBUG
  cv::Mat current_image = cv::imread(Filename(), cv::IMREAD_COLOR);
  for (auto mp: map_points_) {
    cv::circle(current_image,
               cv::Point(features_.keypoints[mp.first].X(), features_.keypoints[mp.first].Y()),
               3,
               cv::Scalar(0, 255, 0));
  }
  cv::imshow("current", current_image);
  cv::waitKey(1);
#endif
  return true;*/
}

void MonocularFrame::AddMapPoint(Observation * observation) {
  assert(observation->GetFrame() == this);
  auto mon_observation = dynamic_cast<MonocularObservation *> (observation);
  assert(nullptr != mon_observation);
  map_points_[mon_observation->GetFeatureId()] = mon_observation->GetMapPoint();
}

MonocularFrame::~MonocularFrame() {
//  for (auto & mp_id: map_points_) {
//    mp_id.second->EraseObservation(this);
//    if (mp_id.second->Observations().empty())
//      delete mp_id.second;
//  }
}

void MonocularFrame::CreateNewMapPoints(FrameBase * other) {
  ComputeBow();
  CovisibilityGraph().Update();

  std::unordered_set<map::MapPoint *> existing_local_map_points;
  ListMapPoints(existing_local_map_points);

  logging::RetrieveLogger()->debug("LM: Initial local map point count: {}", existing_local_map_points.size());

  if (other->Type() != Type()) {
    throw std::runtime_error("Matching Stereo With Monocular is not implemented yet");
  }

  auto other_frame = dynamic_cast<MonocularFrame *>(other);
  if (other_frame == this)
    return;

  other_frame->ComputeBow();

  features::matching::SNNMatcher<features::matching::iterators::BowToIterator> bow_matcher(0.6, 50);
  features::matching::iterators::BowToIterator bow_it_begin(features_.bow_container.feature_vector.begin(),
                                                            &features_.bow_container.feature_vector,
                                                            &other_frame->features_.bow_container.feature_vector,
                                                            &features_,
                                                            &other_frame->features_,
                                                            &map_points_,
                                                            &other_frame->map_points_,
                                                            false,
                                                            false);

  features::matching::iterators::BowToIterator bow_it_end(features_.bow_container.feature_vector.end(),
                                                          &features_.bow_container.feature_vector,
                                                          &other_frame->features_.bow_container.feature_vector,
                                                          &features_,
                                                          &other_frame->features_,
                                                          &map_points_,
                                                          &other_frame->map_points_,
                                                          false,
                                                          false);
  geometry::Pose relative_pose, inverse_relative_pose;
  geometry::utils::ComputeRelativeTransformation(pose_, other_frame->pose_, relative_pose);
  inverse_relative_pose = relative_pose.GetInversePose();

  features::matching::validators::TriangulationValidator
      validator(&features_, &other_frame->features_, &relative_pose, feature_extractor_, camera_->FxInv());
  bow_matcher.AddValidator(&validator);

  decltype(bow_matcher)::MatchMapType matches;
  bow_matcher.MatchWithIteratorV2(bow_it_begin, bow_it_end, feature_extractor_.get(), matches);

  logging::RetrieveLogger()->debug("LM: SNNMatcher found {} matches between {} and {}",
                                   matches.size(),
                                   other_frame->Id(),
                                   Id());
  unsigned newly_created_mps = 0;
  this->recent_observations_ = decltype(recent_observations_)(new std::unordered_set<Observation *>);
  other_frame->recent_observations_ = decltype(recent_observations_)(new std::unordered_set<Observation *>);

  for (auto match: matches) {
    precision_t parallax;
    TPoint3D triangulated;
    if (!geometry::utils::TriangulateAndValidate(other_frame->features_.undistorted_and_unprojected_keypoints[match.second],
                                                 features_.undistorted_and_unprojected_keypoints[match.first],
                                                 relative_pose,
                                                 camera_->FxInv(),
                                                 other_frame->GetCamera()->FxInv(),
                                                 constants::PARALLAX_THRESHOLD,
                                                 parallax,
                                                 triangulated))
      continue;

    precision_t min_invariance_distance, max_invariance_distance;
    feature_extractor_->ComputeInvariantDistances(triangulated,
                                                  features_.keypoints[match.first],
                                                  max_invariance_distance,
                                                  min_invariance_distance);
    auto map_point = new map::MapPoint(inverse_relative_pose.Transform(triangulated), max_invariance_distance, min_invariance_distance);
    auto this_observation = new MonocularObservation(map_point, this, match.first);
    auto other_observation = new MonocularObservation(map_point, other_frame, match.second);
    this->recent_observations_->insert(this_observation);
    other_frame->recent_observations_->insert(other_observation);
    map_point->IncreaseVisible();
    map_point->IncreaseVisible();
    map_point->IncreaseFound();
    map_point->IncreaseFound();
    ++newly_created_mps;
  }
  logging::RetrieveLogger()->debug("LM: Created {} new map_points between frames {} and {}",
                                   newly_created_mps,
                                   other_frame->Id(),
                                   Id());
}

void MonocularFrame::ListMapPoints(unordered_set<map::MapPoint *> & out_map_points) const {
  for (auto mp: map_points_)
    out_map_points.insert(mp.second);
}

void MonocularFrame::EraseMapPoint(map::MapPoint * map_point) {
  auto it = map_point->Observations().find(this);
  assert(it != map_point->Observations().end());
  map_points_.erase(dynamic_cast<MonocularObservation *>(it->second)->GetFeatureId());
}

}
}  // namespace orb_slam3