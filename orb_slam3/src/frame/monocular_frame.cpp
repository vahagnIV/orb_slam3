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
  assert(0 != from_frame);
  features::matching::SNNMatcher matcher(0.9, 50);
  features::matching::iterators::AreaToIterator begin(0, &features_, &from_frame->features_, 100);
  features::matching::iterators::AreaToIterator end(features_.Size(), &features_, &from_frame->features_, 100);

  std::unordered_map<std::size_t, std::size_t> matches;
  matcher.MatchWithIteratorV2(begin, end, feature_extractor_.get(), matches);
  logging::RetrieveLogger()->debug("Orientation validator discarderd {} matches",
                                   features::matching::OrientationValidator
                                       (features_.keypoints, from_frame->features_.keypoints).Validate(matches));

  logging::RetrieveLogger()->debug("Link: SNN Matcher returned {} matches between frames {} and {}.",
                                   matches.size(),
                                   Id(),
                                   other->Id());
  if (matches.size() < 100) {
    logging::RetrieveLogger()->debug("Not enough matches. Skipping");
    return false;
  }

  geometry::TwoViewReconstructor reconstructor(5, camera_->FxInv());
  std::unordered_map<size_t, TPoint3D> points;
  std::unordered_set<size_t> inliers;
  if (!reconstructor.Reconstruct(features_.undistorted_and_unprojected_keypoints,
                                 from_frame->features_.undistorted_and_unprojected_keypoints,
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

  typedef std::unordered_map<std::size_t, std::size_t>::const_iterator I;
  for (I i = matches.begin(); i != matches.end(); ++i) {
    if (inliers.find(i->first) == inliers.end())
      continue;

    if (from_frame->map_points_.find(i->second) == from_frame->map_points_.end()) {
      precision_t max_invariance_distance, min_invariance_distance;
      feature_extractor_->ComputeInvariantDistances(pose_.R * points[i->first] + pose_.T,
                                                    features_.keypoints[i->first],
                                                    max_invariance_distance,
                                                    min_invariance_distance);
      auto map_point = new map::MapPoint(points[i->first], max_invariance_distance, min_invariance_distance);
      AddMapPoint(map_point, i->first);

      from_frame->AddMapPoint(map_points_[i->first], i->second);
      map_point->AddObservation(this, i->first);
      map_point->AddObservation(from_frame, i->second);
      map_point->Refresh(feature_extractor_);
    } else {
      // TODO: do the contistency check
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
    rk->setDelta(std::sqrt(5.99) * camera_->FxInv());
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
    AddMapPoint(map_point, match.first);
  }
  std::unordered_set<std::size_t> inliers;
#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << "TWRKF: Tracking optimization. Pose before optimization\n";
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif

  OptimizePose(inliers);

  if (inliers.size() <= 4) {
    logging::RetrieveLogger()->debug("TWRKF: not enough inliers after optimization");
    return false;
  }
#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << "TWRKF: Tracking optimization. Pose after optimization\n";
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif
  for (const auto & match: matches) {
    if (inliers.find(match.first) == inliers.end()) {
      map_points_.erase(match.first);
    }
  }
  return true;
}

void MonocularFrame::ComputeBow() {
  features_.ComputeBow();
}

void MonocularFrame::InitializeOptimizer(g2o::SparseOptimizer & optimizer) {
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>
      linearSolver(new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());

  std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));

  auto * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
//  optimizer.setVerbose(true);
}

void MonocularFrame::OptimizePose(std::unordered_set<std::size_t> & out_inliers) {

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
        Eigen::Matrix<double, 2, 2>::Identity() / information_coefficient);
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

//  optimizer.setVerbose(true);

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
                                    std::unordered_map<std::size_t, std::size_t> & out_matches,
                                    bool self_keypoint_exists,
                                    bool reference_kf_keypoint_exists) {
  features::matching::SNNMatcher bow_matcher(0.7, 50);
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
      (features_.keypoints, reference_kf->features_.keypoints).Validate(out_matches);

}

void MonocularFrame::ListMapPoints(std::unordered_set<map::MapPoint *> & out_map_points) const {
  for (auto mp: map_points_) {
    if (mp.second->IsValid())
      out_map_points.insert(mp.second);
  }
}

void MonocularFrame::FindNewMapPointMatches(MonocularFrame * keyframe,
                                            std::unordered_map<std::size_t, std::size_t> & out_matches) {

  ComputeBow();
  keyframe->ComputeBow();

  if (BaselineIsNotEnough(keyframe)) {
    logging::RetrieveLogger()->debug("Baseline between frames  {} and {} is not enough", Id(), keyframe->Id());
    return;
  }
  geometry::Pose relative_pose;
  ComputeMatches(keyframe, out_matches, false, false);
  for (auto & match: out_matches) {
    assert(keyframe->map_points_.find(match.second) == keyframe->map_points_.end());
    assert(map_points_.find(match.first) == map_points_.end());
  }
  std::stringstream stringstream1;
  stringstream1 << "/data/tmp/test-match/";
  stringstream1 << Id() << "-" << keyframe->Id() << ".jpg";
  cv::imwrite(stringstream1.str(),
              debug::DrawMatches(Filename(), keyframe->Filename(), out_matches, features_, keyframe->features_));
  logging::RetrieveLogger()->debug("Local mapper: SNN Matcher found {} new matches between {} and {}",
                                   out_matches.size(),
                                   Id(),
                                   keyframe->Id());

}

void MonocularFrame::CreateNewMpPoints(MonocularFrame * keyframe,
                                       const std::unordered_map<std::size_t, std::size_t> & matches) {
  unsigned new_map_point_count = 0;
  geometry::Pose relative_pose;
  geometry::utils::ComputeRelativeTransformation(pose_, keyframe->pose_, relative_pose);
  for (auto & match: matches) {
    assert(keyframe->map_points_.find(match.second) == keyframe->map_points_.end());
    TPoint3D pt;
    precision_t parallax;

    map::MapPoint * map_point;
    auto item = map_points_.find(match.first);
    if (item == map_points_.end()) {
      if (!geometry::utils::TriangulateAndValidate(features_.undistorted_and_unprojected_keypoints[match.first],
                                                   keyframe->features_.undistorted_and_unprojected_keypoints[match.second],
                                                   relative_pose,
                                                   camera_->FxInv(),
                                                   keyframe->camera_->FxInv(),
                                                   constants::PARALLAX_THRESHOLD,
                                                   parallax,
                                                   pt))
        continue;

      precision_t max_invariance_distance, min_invariance_distance;
      feature_extractor_->ComputeInvariantDistances(relative_pose.R * pt + relative_pose.T,
                                                    features_.keypoints[match.first],
                                                    max_invariance_distance,
                                                    min_invariance_distance);
      map_point = new map::MapPoint(keyframe->GetInversePose()->Transform(pt),
                                    max_invariance_distance,
                                    min_invariance_distance);
      ++new_map_point_count;
      AddMapPoint(map_point, match.first);
      map_point->AddObservation(this, match.first);
    } else
      map_point = item->second;
    map_point->AddObservation(keyframe, match.second);
    keyframe->AddMapPoint(map_point, match.second);
  }

  logging::RetrieveLogger()->debug("LM: Created {} new map_points between frames {} and {}",
                                   new_map_point_count,
                                   keyframe->Id(),
                                   Id());
}

optimization::edges::SE3ProjectXYZPose * MonocularFrame::CreateEdge(map::MapPoint * map_point, MonocularFrame * frame) {
  // TODO: this assumes that the camera is the same for all frames
  static const precision_t delta_mono = constants::HUBER_MONO_DELTA * camera_->FxInv();
  auto edge = new optimization::edges::SE3ProjectXYZPose(frame->camera_.get());
  auto measurement = frame->features_.undistorted_and_unprojected_keypoints[map_point->Observations()[frame]];
//  std::cout << measurement;
  edge->setMeasurement(Eigen::Map<Eigen::Matrix<double,
                                                2,
                                                1>>(measurement.data()));
  edge->setInformation(Eigen::Matrix2d::Identity());
  auto rk = new g2o::RobustKernelHuber;
  rk->setDelta(delta_mono);
  edge->setRobustKernel(rk);
  return edge;
}

bool MonocularFrame::FindNewMapPoints() {
//  CovisibilityGraph().Update();
//  std::unordered_set<frame::FrameBase *> neighbour_keyframes = CovisibilityGraph().GetCovisibleKeyFrames(20);
// TODO: change to covisibility graph


  std::unordered_set<frame::FrameBase *> neighbour_keyframes;
  std::unordered_set<map::MapPoint *> existing_local_map_points, all_existing_points;
  ListMapPoints(existing_local_map_points);
  ListLocalKeyFrames(existing_local_map_points, neighbour_keyframes);
  ListAllMapPoints(neighbour_keyframes, all_existing_points);

  unsigned min_new_map_point_id = Identifiable::GetNextId();
  for (auto mp: map_points_)
    mp.second->AddObservation(this, mp.first);

  for (frame::FrameBase * frame : neighbour_keyframes) {

    if (frame->Type() != Type()) {
      continue;
    }

    auto keyframe = dynamic_cast<MonocularFrame *>(frame);
    std::unordered_map<std::size_t, std::size_t> matches;
    FindNewMapPointMatches(keyframe, matches);
    for (auto & match: matches) {
      assert(keyframe->map_points_.find(match.second) == keyframe->map_points_.end());
      assert(map_points_.find(match.first) == map_points_.end());
    }
    std::unordered_set<std::size_t> validation_set;
    for (auto match: matches) {
      assert(validation_set.find(match.second) == validation_set.end());
      validation_set.insert(match.second);
    }
    logging::RetrieveLogger()->debug("LM: SNNMatcher found {} matches between {} and {}",
                                     matches.size(),
                                     frame->Id(),
                                     Id());
    CreateNewMpPoints(keyframe, matches);
  }

  // We will reuse neighbour_keyframes bearing in mind that the initial frame can still be there
  g2o::SparseOptimizer optimizer;
  InitializeOptimizer(optimizer);

  std::unordered_set<FrameBase *> fixed_frames;
  this->FixedFrames(all_existing_points, neighbour_keyframes, fixed_frames);

  if (optimizer.vertex(Id())) {
    std::cout << "Pizdec naxuy blyad' 0" << std::endl;
  }
  optimizer.addVertex(this->CreatePoseVertex());

  std::unordered_map<std::size_t, MonocularFrame *> frame_map{{Id(), this}};
  for (auto & frame: neighbour_keyframes) {
    frame_map[frame->Id()] = dynamic_cast<MonocularFrame *>(frame);
    auto vertex = frame->CreatePoseVertex();
    if (optimizer.vertex(vertex->id())) {
      std::cout << "Pizdec naxuy blyad' 1" << std::endl;
    }
    optimizer.addVertex(vertex);
    vertex->setFixed(fixed_frames.find(frame) != fixed_frames.end());
  }

  size_t last_id = Identifiable::GetNextId();

  std::unordered_map<std::size_t, map::MapPoint *> mp_map;
  std::unordered_map<map::MapPoint *, std::list<optimization::edges::SE3ProjectXYZPose *>> mp_edges;
  for (auto mp_id: map_points_) {
    auto map_point = mp_id.second;
    mp_map[map_point->Id()] = map_point;
    auto vertex = map_point->CreateVertex();
    vertex->setMarginalized(true);
    if (optimizer.vertex(vertex->id())) {
      std::cout << "Pizdec naxuy blyad' 2" << std::endl;
    }
    optimizer.addVertex(vertex);

    for (const auto & observation: map_point->Observations()) {
      auto obs_frame = dynamic_cast<MonocularFrame *>(observation.first);
      if (nullptr == obs_frame)
        continue;
      auto edge = CreateEdge(map_point, obs_frame);
      edge->setVertex(0, optimizer.vertex(observation.first->Id()));
      edge->setVertex(1, vertex);
      edge->setId(++last_id);
      mp_edges[map_point].push_back(edge);
      optimizer.addEdge(edge);
    }
  }
  optimizer.initializeOptimization();
  optimizer.optimize(5);

  // Collect frame positions
  SetPosition(dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(Id()))->estimate());
  for (auto f: neighbour_keyframes) {
    auto frame = dynamic_cast<MonocularFrame *>(f);
    auto frame_pose_vertex = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(f->Id()));
    if (!frame_pose_vertex->fixed())
      frame->SetPosition(frame_pose_vertex->estimate());
  }

  const precision_t allowed_max_error = constants::MONO_CHI2 * camera_->FxInv() * camera_->FxInv();

  auto mp_id = map_points_.begin();
  int counter = 0;
  while (mp_id != map_points_.end()) {
//    std::ofstream of("/home/vahagn/Desktop/orb_slam_dump/map_begin" + std::to_string(counter++) + ".txt");
//    for (auto mp_id: map_points_) {
//      of << mp_id.first << " " << mp_id.second << std::endl;
//    }
//    of.close();
    map::MapPoint * map_point = mp_id->second;
    auto mp_vertex = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(map_point->Id()));
    map_point->SetPosition(mp_vertex->estimate());

    bool advance_iterator = true;
    for (auto edge: mp_edges[map_point]) {
      assert(edge->vertex(1)->id() == (int) map_point->Id());
      MonocularFrame * frame = frame_map[edge->vertex(0)->id()];

      if (edge->chi2() > allowed_max_error) {

        if (frame != this) {
          frame->map_points_.erase(map_point->Observations()[frame]);
        } else {
          assert(advance_iterator);
          advance_iterator = false;
          mp_id = map_points_.erase(mp_id);
        }
        map_point->Observations().erase(frame);
      }
    }

    if (map_point->Observations().size() <= 1) {
      for (auto obs: map_point->Observations()) {
        dynamic_cast<MonocularFrame *>(obs.first)->map_points_.erase(obs.second);
      }
      delete map_point;
    } else
      map_point->Refresh(feature_extractor_);

    if (advance_iterator)
      ++mp_id;

  }

  return true;
}

void MonocularFrame::AddMapPoint(map::MapPoint * map_point, size_t feature_id) {
  assert(map_points_.find(feature_id) == map_points_.end());
  assert(!MapPointExists(map_point));
  map_points_[feature_id] = map_point;
}

bool MonocularFrame::MapPointExists(const map::MapPoint * map_point) const {
  for (auto mp_id: map_points_)
    if (mp_id.second == map_point) {
      std::cout << "MP Exists " << mp_id.first << std::endl;
      return true;
    }
  return false;
}

bool MonocularFrame::TrackLocalMap(const std::shared_ptr<frame::FrameBase> & last_keyframe) {

  std::unordered_set<map::MapPoint *> all_candidate_map_points;
  std::unordered_set<FrameBase *> local_frames;
  FrameBase * max_covisible_frame;
  if (nullptr == (max_covisible_frame = last_keyframe->ListLocalKeyFrames(all_candidate_map_points, local_frames))) {
    return false;
  }

  std::unordered_set<map::MapPoint *> existing_local_map_points;
  ListMapPoints(existing_local_map_points);

  logging::RetrieveLogger()->debug("TLM: local_frame: {}, local_map_point: {}, current_map_points: {}",
                                   local_frames.size(),
                                   existing_local_map_points.size(),
                                   all_candidate_map_points.size());

  features::matching::iterators::ProjectionSearchIterator begin
      (all_candidate_map_points.begin(),
       all_candidate_map_points.end(),
       &existing_local_map_points,
       &map_points_,
       &features_,
       &pose_,
       camera_.get(),
       feature_extractor_.get());

  features::matching::iterators::ProjectionSearchIterator end
      (all_candidate_map_points.end(),
       all_candidate_map_points.end(),
       &existing_local_map_points,
       &map_points_,
       &features_,
       &pose_,
       camera_.get(),
       feature_extractor_.get());

  logging::RetrieveLogger()->debug("TLM: Local mp point count: {}", existing_local_map_points.size());

  features::matching::SNNMatcher matcher(0.8, 100);
  std::unordered_map<map::MapPoint *, std::size_t> matches;
  matcher.MatchWithIteratorV2(begin, end, feature_extractor_.get(), matches);
  // TODO: set asserts

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

  logging::RetrieveLogger()->info("TLM: Found {} matches", matches.size());

  for (auto match: matches) {
    AddMapPoint(match.first, match.second);
  }

  std::unordered_set<std::size_t> inliers;
  if (map_points_.size() < 20) {
    map_points_.clear();
    logging::RetrieveLogger()->debug("TLM: Not enough map points for optimization");
    return false;
  }
#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << "TLM: Tracking optimization. Pose before optimization\n";
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif

  OptimizePose(inliers);
  if (inliers.size() < 15) {
    logging::RetrieveLogger()->debug("TLM: Not enough inliers after optimization");
  }

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
      mp_it = map_points_.erase(mp_it);
    else {
      ++mp_it;
    }
  }

  if (map_points_.size() < 15) {
    map_points_.clear();
    return false;
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

  return true;
}

MonocularFrame::~MonocularFrame() {
//  for (auto & mp_id: map_points_) {
//    mp_id.second->EraseObservation(this);
//    if (mp_id.second->Observations().empty())
//      delete mp_id.second;
//  }
}

}
}  // namespace orb_slam3