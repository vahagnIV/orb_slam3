//
// Created by vahagn on 1/23/21.
//

// === g2o ===
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>

// == orb-slam3 ===
#include <frame/monocular_frame.h>
#include <constants.h>
#include <features/matching/second_nearest_neighbor_matcher.h>
#include <features/matching/iterators/bow_to_iterator.h>
#include <features/matching/validators/bow_match_tracking_validator.h>
#include <features/matching/validators/bow_match_local_mapping_validator.h>
#include <features/matching/orientation_validator.h>
#include <geometry/two_view_reconstructor.h>
#include <geometry/utils.h>
#include <optimization/edges/se3_project_xyz_pose.h>
#include <optimization/edges/se3_project_xyz_pose_only.h>
#include <optimization/bundle_adjustment.h>
#include <logging.h>

#include <features/matching/iterators/area_to_iterator.h>
#include <features/matching/iterators/projection_search_iterator.h>

#include <debug/debug_utils.h>

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

bool MonocularFrame::Link(const std::shared_ptr<FrameBase> & other) {
  logging::RetrieveLogger()->info("Linking frame {} with {}", Id(), other->Id());
  if (other->Type() != Type()) {
    logging::RetrieveLogger()->warn("Frames {} and {} have different types. Could not link", Id(), other->Id());
    return false;
  }
  MonocularFrame * from_frame = dynamic_cast<MonocularFrame *>(other.get());
  features::matching::SNNMatcher matcher(0.9, 50);
  features::matching::iterators::AreaToIterator begin(0, &features_, &from_frame->features_, 100);
  features::matching::iterators::AreaToIterator end(features_.Size(), &features_, &from_frame->features_, 100);
  std::unordered_map<std::size_t, std::size_t> matches2;
  matcher.MatchWithIteratorV2(begin, end, feature_extractor_.get(), matches2);
  logging::RetrieveLogger()->debug("Orientation validator discarderd {} matches",
                                   features::matching::OrientationValidator
                                       (features_.keypoints, from_frame->features_.keypoints).Validate(matches2));

  logging::RetrieveLogger()->debug("Link: SNN Matcher returned {} matches between frames {} and {}.",
                                   matches2.size(),
                                   Id(),
                                   other->Id());
  if (matches2.size() < 100) {
    logging::RetrieveLogger()->debug("Not enough matches. Skipping");
    return false;
  }

  std::vector<features::Match> matches;
  std::transform(matches2.begin(),
                 matches2.end(),
                 std::back_inserter(matches),
                 [](const std::pair<std::size_t, std::size_t> & m) { return features::Match(m.first, m.second); });

  geometry::TwoViewReconstructor reconstructor(5, camera_->FxInv());
  std::vector<TPoint3D> points;
  std::vector<bool> inliers;
  if (!reconstructor.Reconstruct(features_.undistorted_keypoints,
                                 from_frame->features_.undistorted_keypoints,
                                 matches,
                                 pose_,
                                 points,
                                 inliers)) {
    logging::RetrieveLogger()->info("LINKING: Could not reconstruct points between frames {} and {}. Skipping...",
                                    Id(),
                                    other->Id());
    return false;
  }

  cv::imshow("linked matches:",
             debug::DrawMatches(Filename(), other->Filename(), matches, features_, from_frame->features_));

  for (size_t i = 0; i < matches.size(); ++i) {
    if (!inliers[i])
      continue;
    const features::Match & match = matches[i];

    if (from_frame->map_points_.find(match.from_idx) != from_frame->map_points_.end()) {
      // TODO: do the contistency check
    } else {
      precision_t max_invariance_distance, min_invariance_distance;
      feature_extractor_->ComputeInvariantDistances(pose_.R * points[i] + pose_.T,
                                                    features_.keypoints[match.to_idx],
                                                    max_invariance_distance,
                                                    min_invariance_distance);
      auto map_point = new map::MapPoint(points[i], max_invariance_distance, min_invariance_distance);
      map_points_[match.to_idx] = map_point;
      from_frame->map_points_[match.from_idx] = map_points_[match.to_idx];
      map_point->AddObservation(this, matches[i].to_idx);
      map_point->AddObservation(from_frame, matches[i].from_idx);
      map_point->Refresh(feature_extractor_);
    }
  }
  from_frame->CovisibilityGraph().Update();
  this->CovisibilityGraph().Update();

#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug("LINKING: Frame {} position before BA:", Id());
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif

  optimization::BundleAdjustment({this, from_frame}, 20);
#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug("LINKING: Frame {} position after BA:", Id());
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif
  return true;

}

void MonocularFrame::AppendDescriptorsToList(size_t feature_id,
                                             std::vector<features::DescriptorType> & out_descriptor_ptr) const {

  out_descriptor_ptr.emplace_back(features_.descriptors.row(feature_id));

}

void MonocularFrame::AppendToOptimizerBA(g2o::SparseOptimizer & optimizer, size_t & next_id) {
  g2o::VertexSE3Expmap * pose = CreatePoseVertex();
  optimizer.addVertex(pose);
  for (auto & mp_id:map_points_) {
    if (nullptr == mp_id.second)
      continue;
    map::MapPoint * map_point = mp_id.second;
    size_t feature_id = mp_id.first;

    g2o::VertexPointXYZ * mp;
    if (nullptr == optimizer.vertex(map_point->Id())) {
      mp = map_point->CreateVertex();
      optimizer.addVertex(mp);
    } else
      mp = dynamic_cast< g2o::VertexPointXYZ *>(optimizer.vertex(map_point->Id()));

    auto edge = new optimization::edges::SE3ProjectXYZPose(camera_.get());
    edge->setVertex(0, pose);
    edge->setVertex(1, mp);
    edge->setId(next_id++);
    edge->setInformation(Eigen::Matrix2d::Identity());
    g2o::RobustKernelHuber * rk = new g2o::RobustKernelHuber;
    edge->setRobustKernel(rk);
    rk->setDelta(std::sqrt(5.99));
    HomogenousPoint measurement;
    camera_->UnprojectPoint(features_.keypoints[feature_id].pt, measurement);
    TPoint2D m;
    m << measurement[0], measurement[1];
    edge->setMeasurement(m);
    optimizer.addEdge(edge);
  }
}

void MonocularFrame::CollectFromOptimizerBA(g2o::SparseOptimizer & optimizer) {
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *> (optimizer.vertex(Id()));
  SetPosition(pose->estimate());
  for (auto mp: map_points_) {
    if (nullptr == mp.second)
      continue;
    if (mp.second->Observations().begin()->first->Id() == Id()) {
      auto position = dynamic_cast<g2o::VertexPointXYZ *> (optimizer.vertex(mp.second->Id()));
      mp.second->SetPosition(position->estimate());
      mp.second->Refresh(feature_extractor_);
    }
  }
}

TVector3D MonocularFrame::GetNormal(const TPoint3D & point) const {
  TPoint3D normal = point - inverse_pose_.T;
  normal.normalize();
  return normal;
}

bool MonocularFrame::TrackWithReferenceKeyFrame(const std::shared_ptr<FrameBase> & reference_keyframe) {
  logging::RetrieveLogger()->info("TWRKF: Tracking frame {} with reference keyframe {}",
                                  Id(),
                                  reference_keyframe->Id());
  if (reference_keyframe->Type() != Type()) {
    logging::RetrieveLogger()->warn("Frames {} and {} have different types", Id(), reference_keyframe->Id());
    return false;
  }
  auto reference_kf = dynamic_cast<MonocularFrame *>(reference_keyframe.get());

  // Ensure bows are computed
  reference_kf->ComputeBow();
  ComputeBow();

  features::matching::SNNMatcher bow_matcher(0.7, 50);
  std::unordered_map<std::size_t, std::size_t> matches;
  features::matching::iterators::BowToIterator bow_it_begin(features_.bow_container.feature_vector.begin(),
                                                            &features_.bow_container.feature_vector,
                                                            &reference_kf->features_.bow_container.feature_vector,
                                                            &features_,
                                                            &reference_kf->features_,
                                                            &map_points_,
                                                            &reference_kf->map_points_,
                                                            false,
                                                            true);

  features::matching::iterators::BowToIterator bow_it_end(features_.bow_container.feature_vector.end(),
                                                          &features_.bow_container.feature_vector,
                                                          &reference_kf->features_.bow_container.feature_vector,
                                                          &features_,
                                                          &reference_kf->features_,
                                                          &map_points_,
                                                          &reference_kf->map_points_,
                                                          false,
                                                          true);

  bow_matcher.MatchWithIteratorV2(bow_it_begin, bow_it_end, feature_extractor_.get(), matches);

  logging::RetrieveLogger()->info("TWRKF: SNNMatcher returned {} matches for frames {} and {}",
                                  matches.size(),
                                  Id(),
                                  reference_keyframe->Id());
  if (matches.size() < 15) {
    return false;
  }

  cv::imshow("TrackWithReferenceKeyframe",
             debug::DrawMatches(Filename(), reference_kf->Filename(), matches, features_, reference_kf->GetFeatures()));
  cv::waitKey(1);

  for (const auto & match: matches) {
    auto map_point = reference_kf->map_points_[match.second];
    map_points_[match.first] = map_point;
  }
  std::unordered_set<std::size_t> inliers;
//  SetPosition(*(reference_kf->GetPose()));
#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << "TWRKF: Tracking optimization. Pose before optimization\n";
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif
  OptimizePose(inliers);
#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << "TWRKF: Tracking optimization. Pose after optimization\n";
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif
  for (const auto & match: matches) {
    if (inliers.find(match.first) != inliers.end()) {
      /*map_points_[match.to_idx]->AddObservation(this, match.to_idx);
      map_points_[match.to_idx]->Refresh();*/
    } else
      map_points_.erase(match.second);
  }
  //covisibility_connections_.Update();
  //reference_kf->covisibility_connections_.Update();
  return map_points_.size() > 10;
}

void MonocularFrame::ComputeBow() {
  features_.ComputeBow();
}

void MonocularFrame::OptimizePose(std::unordered_set<std::size_t> & out_inliers) {
  static const precision_t delta_mono = std::sqrt(5.991);
  static const precision_t chi2[4] = {5.991, 5.991, 5.991, 5.991};
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>
      linearSolver(new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());

  std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));

  auto * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
  g2o::VertexSE3Expmap * pose = CreatePoseVertex();
  optimizer.addVertex(pose);
  size_t last_id = Identifiable::GetNextId();
  std::unordered_map<optimization::edges::SE3ProjectXYZPoseOnly *, std::size_t> edges;

  for (auto mp_id:map_points_) {
    map::MapPoint * map_point = mp_id.second;
    size_t feature_id = mp_id.first;
    if (nullptr == map_point) {
      exit(2);
      continue;
    }
    out_inliers.insert(feature_id);
    auto edge = new optimization::edges::SE3ProjectXYZPoseOnly(camera_.get(), map_point->GetPosition());
    edge->setVertex(0, pose);
    edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());
    edge->setId(last_id++);
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    edge->robustKernel()->setDelta(delta_mono);
    HomogenousPoint measurement;
    camera_->UnprojectPoint(features_.keypoints[feature_id].pt, measurement);
    edge->setMeasurement(Eigen::Map<Eigen::Matrix<double, 2, 1>>(measurement.data()));
    optimizer.addEdge(edge);
    edges[edge] = feature_id;
  }

  optimizer.initializeOptimization(0);
//  optimizer.setVerbose(true);

  for (int i = 0; i < 4 && !out_inliers.empty(); ++i) {
    pose->setEstimate(pose_.GetQuaternion());
    if (out_inliers.empty())
      return;
    optimizer.optimize(10);
    for (auto edge: edges) {

      if (out_inliers.find(edge.second) == out_inliers.end()) { // If  the edge was not included in the optimization
        edge.first->computeError();
      }
      if (edge.first->chi2() < chi2[i]) {
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

  SetPosition(pose->estimate());
}

precision_t MonocularFrame::ComputeMedianDepth() const {
  const g2o::SE3Quat pose_quat(pose_.R, pose_.T);
  std::vector<precision_t> depths(map_points_.size());
  std::transform(map_points_.begin(),
                 map_points_.end(),
                 depths.begin(),
                 [&pose_quat](const std::pair<size_t, map::MapPoint *> & mp_id) -> precision_t {
                   return pose_quat.map(mp_id.second->GetPosition())[2];
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
                                    std::unordered_map<std::size_t, std::size_t> & out_matches) {
  features::matching::SNNMatcher bow_matcher(0.7, 50);
  std::unordered_map<std::size_t, std::size_t> matches;
  features::matching::iterators::BowToIterator bow_it_begin(features_.bow_container.feature_vector.begin(),
                                                            &features_.bow_container.feature_vector,
                                                            &reference_kf->features_.bow_container.feature_vector,
                                                            &features_,
                                                            &reference_kf->features_,
                                                            &map_points_,
                                                            &reference_kf->map_points_,
                                                            false,
                                                            false);

  features::matching::iterators::BowToIterator bow_it_end(features_.bow_container.feature_vector.end(),
                                                          &features_.bow_container.feature_vector,
                                                          &reference_kf->features_.bow_container.feature_vector,
                                                          &features_,
                                                          &reference_kf->features_,
                                                          &map_points_,
                                                          &reference_kf->map_points_,
                                                          false,
                                                          false);

  bow_matcher.MatchWithIteratorV2(bow_it_begin, bow_it_end, feature_extractor_.get(), matches);
  features::matching::OrientationValidator
      (features_.keypoints, reference_kf->features_.keypoints).Validate(matches);

}

void MonocularFrame::ListMapPoints(std::unordered_set<map::MapPoint *> & out_map_points) const {
  for (auto mp: map_points_) {
    if (mp.second->IsValid())
      out_map_points.insert(mp.second);
  }
}

void MonocularFrame::FindNewMapPoints() {
  CovisibilityGraph().Update();
  std::unordered_set<frame::FrameBase *> neighbour_keyframes = CovisibilityGraph().GetCovisibleKeyFrames(20);

  for (frame::FrameBase * frame : neighbour_keyframes) {


    if (frame->Type() != Type())
      continue;
    auto keyframe = dynamic_cast<MonocularFrame *>(frame);
    ComputeBow();
    keyframe->ComputeBow();

    if (BaselineIsNotEnough(keyframe)) {
      logging::RetrieveLogger()->debug("Baseline between frames  {} and {} is not enough", Id(), keyframe->Id());
      continue;
    }
    std::unordered_map<std::size_t, std::size_t> matches;
    geometry::Pose relative_pose;
    ComputeMatches(keyframe, matches);
    logging::RetrieveLogger()->debug("Local mapper found {} new map-points between {} and {}",
                                     matches.size(),
                                     Id(),
                                     keyframe->Id());

    for (auto & match: matches) {

      TPoint3D pt;
      geometry::utils::Triangulate(relative_pose,
                                   keyframe->features_.undistorted_keypoints[match.second],
                                   features_.undistorted_keypoints[match.first],
                                   pt);
      precision_t max_invariance_distance, min_invariance_distance;
      feature_extractor_->ComputeInvariantDistances(relative_pose.R * pt + relative_pose.T,
                                                    features_.keypoints[match.first],
                                                    max_invariance_distance,
                                                    min_invariance_distance);
      auto mp = new map::MapPoint(keyframe->pose_.R.transpose()
                                      * (pt - keyframe->pose_.T), max_invariance_distance, min_invariance_distance);
      map_points_[match.first] = mp;
      keyframe->map_points_[match.second] = mp;
      mp->AddObservation(this, match.first);
      mp->AddObservation(keyframe, match.second);
      mp->Refresh(feature_extractor_);
      covisibility_connections_.Update();
      keyframe->covisibility_connections_.Update();
    }
  }

  // We will reuse neighbour_keyframes bearing in mind that the initial frame can still be there
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>
      linearSolver(new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());

  std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));

  g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
  std::unordered_set<map::MapPoint *> map_points;
  this->ListAllMapPoints(neighbour_keyframes, map_points);
  std::unordered_set<FrameBase *> fixed_frames;
  this->FixedFrames(map_points, neighbour_keyframes, fixed_frames);

  std::unordered_map<size_t, FrameBase *> frame_map{{Id(), this}};
  std::unordered_map<size_t, map::MapPoint *> mp_map;

  for (auto & frame: neighbour_keyframes) {
    auto vertex = frame->CreatePoseVertex();
    vertex->setFixed(frame->IsInitial());
    optimizer.addVertex(vertex);
    frame_map[frame->Id()] = frame;
  }

  for (auto & frame: fixed_frames) {
    if (frame->IsInitial())
      continue;
    auto vertex = frame->CreatePoseVertex();
    vertex->setFixed(true);
    optimizer.addVertex(vertex);
  }
  size_t last_id = Identifiable::GetNextId();
  for (auto map_point: map_points) {
    auto vertex = map_point->CreateVertex();
    vertex->setMarginalized(true);
    optimizer.addVertex(vertex);
    mp_map[map_point->Id()] = map_point;
    for (const auto & observation: map_point->Observations()) {
      auto obs_frame = dynamic_cast<MonocularFrame *>(observation.first);
      if (nullptr == obs_frame)
        continue;
      auto edge = new optimization::edges::SE3ProjectXYZPose(obs_frame->camera_.get());
      HomogenousPoint measurement;
      obs_frame->camera_->UnprojectPoint(features_.keypoints[observation.second].pt, measurement);
      edge->setMeasurement(Eigen::Map<Eigen::Matrix<double, 2, 1>>(measurement.data()));
      edge->setId(++last_id);
      edge->setVertex(0, optimizer.vertex(observation.first->Id()));
      edge->setVertex(1, optimizer.vertex(map_point->Id()));
      optimizer.addEdge(edge);
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(5);

  for (auto edge_base: optimizer.edges()) {
    auto edge = dynamic_cast<optimization::edges::SE3ProjectXYZPose *>(edge_base);
    auto map_point_pose = dynamic_cast<g2o::VertexPointXYZ *>(edge->vertex(1));
    auto frame_pose = dynamic_cast<g2o::VertexSE3Expmap *>(edge->vertex(0));
    map::MapPoint * mp = mp_map[map_point_pose->id()];
    FrameBase * frame_base = frame_map[frame_pose->id()];
    if (nullptr == frame_base || nullptr == mp)
      continue;
    if (edge->chi2() > 5.991 || !edge->IsDepthPositive()) {
      mp->EraseObservation(frame_base);
      dynamic_cast<MonocularFrame *>(frame_base)->map_points_.erase(mp->Observations()[frame_base]);
      if (mp->Observations().empty()) {
        delete mp;
      }
      continue;
    }
    if (map_points.find(mp) != map_points.end()) {
      mp->SetPosition(map_point_pose->estimate());
      map_points.erase(mp);
    }
    if (neighbour_keyframes.find(frame_base) != neighbour_keyframes.end()) {
      frame_base->SetPosition(frame_pose->estimate());
      neighbour_keyframes.erase(frame_base);
    }
  }

  std::stringstream ss;
  ss << pose_.R << std::endl << pose_.T << std::endl;
  logging::RetrieveLogger()->info(ss.str());

}

bool MonocularFrame::TrackLocalMap(const std::shared_ptr<frame::FrameBase> & last_keyframe) {

  std::unordered_set<map::MapPoint *> current_frame_map_points;
  std::unordered_set<FrameBase *> local_frames;
  FrameBase * max_covisible_frame;
  if (nullptr == (max_covisible_frame = last_keyframe->ListLocalKeyFrames(current_frame_map_points, local_frames))) {
    return false;
  }

  std::unordered_set<map::MapPoint *> local_map_points;
  FrameBase::ListAllMapPoints(local_frames, local_map_points);

  logging::RetrieveLogger()->debug("TLM: local_frame: {}, local_map_point: {}, current_map_points: {}",
                                   local_frames.size(),
                                   local_map_points.size(),
                                   current_frame_map_points.size());

  features::matching::iterators::ProjectionSearchIterator begin
      (local_map_points.begin(),
       local_map_points.end(),
       &current_frame_map_points,
       &features_,
       &pose_,
       camera_.get(),
       feature_extractor_.get());
  features::matching::iterators::ProjectionSearchIterator end
      (local_map_points.end(),
       local_map_points.end(),
       &current_frame_map_points,
       &features_,
       &pose_,
       camera_.get(),
       feature_extractor_.get());

  features::matching::SNNMatcher matcher(0.8, 100);
  std::unordered_map<map::MapPoint *, std::size_t> matches;
  matcher.MatchWithIteratorV2(begin, end, feature_extractor_.get(), matches);

  logging::RetrieveLogger()->info("TLM: Found {} maatches", matches.size());

  for (auto match: matches) {
    map_points_[match.second] = match.first;
  }

  std::unordered_set<std::size_t> inliers;
  if (map_points_.size() < 30)
    return false;
#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << "TLM: Tracking optimization. Pose before optimization\n";
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif

  OptimizePose(inliers);

#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << "TLM: Tracking optimization. Pose after optimization\n";
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif

  for (decltype(map_points_)::iterator mp_it = map_points_.begin(); mp_it != map_points_.end();) {
    if (inliers.find(mp_it->first) == inliers.end())
      map_points_.erase(mp_it++);
    else {
//      mp_it->second->AddObservation(this, mp_it->first);
//      mp_it->second->Refresh(feature_extractor_);
      ++mp_it;
    }
  }
  bool ok = map_points_.size() > 20;
  if (!ok) {
    map_points_.clear();
    return ok;
  }
  cv::Mat current_image = cv::imread(Filename(), cv::IMREAD_COLOR);
  for (auto mp: map_points_) {
    cv::circle(current_image,
               cv::Point(features_.keypoints[mp.first].X(), features_.keypoints[mp.first].Y()),
               3,
               cv::Scalar(0, 255, 0));
  }
  cv::imshow("current", current_image);
  cv::waitKey(1);

  return ok;
}

}
}  // namespace orb_slam3