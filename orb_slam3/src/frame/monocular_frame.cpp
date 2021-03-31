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
#include <features/matching/iterators/area_iterator.h>
#include <features/matching/iterators/bow_iterator.h>
#include <features/matching/validators/bow_match_tracking_validator.h>
#include <features/matching/validators/bow_match_local_mapping_validator.h>
#include <features/matching/validators/orientation_validator.h>
#include <geometry/two_view_reconstructor.h>
#include <geometry/utils.h>
#include <optimization/edges/se3_project_xyz_pose.h>
#include <optimization/edges/se3_project_xyz_pose_only.h>
#include <optimization/bundle_adjustment.h>
#include <logging.h>

namespace orb_slam3 {
namespace frame {

MonocularFrame::MonocularFrame(const TImageGray8U & image, TimePoint timestamp,
                               const std::shared_ptr<features::IFeatureExtractor> & feature_extractor,
                               const std::shared_ptr<camera::MonocularCamera> & camera,
                               features::BowVocabulary *vocabulary) :
    FrameBase(timestamp, feature_extractor),
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
  if (other->Type() != Type())
    return false;
  MonocularFrame *from_frame = dynamic_cast<MonocularFrame *>(other.get());
  features::matching::SNNMatcher matcher(0.9);
  features::matching::iterators::AreaIterator area_iterator(from_frame->features_, features_, 300);
  features::matching::validators::OrientationValidator
      orientation_validator(features_.keypoints, from_frame->features_.keypoints);

  matcher.MatchWithIterator(features_.descriptors,
                            from_frame->features_.descriptors,
                            frame_link_.matches,
                            &area_iterator,
                            {},
                            &orientation_validator);

  if (frame_link_.matches.size() < 40)
    return false;

  geometry::TwoViewReconstructor reconstructor(5, camera_->FxInv());
  std::vector<TPoint3D> points;
  if (reconstructor.Reconstruct(features_.undistorted_keypoints,
                                from_frame->features_.undistorted_keypoints,
                                frame_link_.matches,
                                pose_,
                                points,
                                frame_link_.inliers)) {

    // TODO: pass to asolute R,T
    frame_link_.other = other;

    for (size_t i = 0; i < frame_link_.matches.size(); ++i) {
      if (!frame_link_.inliers[i])
        continue;
      const features::Match & match = frame_link_.matches[i];

      if (from_frame->map_points_.find(match.from_idx) != from_frame->map_points_.end()) {
        // TODO: do the contistency check
      } else {

        auto map_point = new map::MapPoint(points[i]);
        map_points_[match.to_idx] = map_point;
        from_frame->map_points_[match.from_idx] = map_points_[match.to_idx];
        map_point->AddObservation(this, frame_link_.matches[i].to_idx);
        map_point->AddObservation(from_frame, frame_link_.matches[i].from_idx);
        map_point->Refresh();
      }
    }
    from_frame->CovisibilityGraph().Update();
    this->CovisibilityGraph().Update();

#ifndef NDEBUG
    {
      std::stringstream ss;
      ss << pose_.R << std::endl << pose_.T << std::endl;
      logging::RetrieveLogger()->debug("Frame {} position before BA:", Id());
      logging::RetrieveLogger()->debug(ss.str());
    }
#endif

    optimization::BundleAdjustment({this, from_frame}, 20);
    // TODO: normalize T
#ifndef NDEBUG
    {
      std::stringstream ss;
      ss << pose_.R << std::endl << pose_.T << std::endl;
      logging::RetrieveLogger()->debug("Frame {} position after BA:", Id());
      logging::RetrieveLogger()->debug(ss.str());
    }
#endif
    return true;
  }

  return false;
}

void MonocularFrame::AppendDescriptorsToList(size_t feature_id,
                                             std::vector<features::DescriptorType> & out_descriptor_ptr) const {

  out_descriptor_ptr.emplace_back(features_.descriptors.row(feature_id));

}

void MonocularFrame::AppendToOptimizerBA(g2o::SparseOptimizer & optimizer, size_t & next_id) {
  g2o::VertexSE3Expmap *pose = CreatePoseVertex();
  optimizer.addVertex(pose);
  for (auto & mp_id:map_points_) {
    if (nullptr == mp_id.second)
      continue;
    map::MapPoint *map_point = mp_id.second;
    size_t feature_id = mp_id.first;

    g2o::VertexPointXYZ *mp;
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
    m << measurement[0],  measurement[1];
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
      mp.second->Refresh();
    }
  }
}

TPoint3D MonocularFrame::GetNormal(const TPoint3D & point) const {
  TPoint3D normal = pose_.T - point;
  normal.normalize();
  return normal;
}

bool MonocularFrame::TrackWithReferenceKeyFrame(const std::shared_ptr<FrameBase> & reference_keyframe) {
  if (reference_keyframe->Type() != Type())
    return false;
  auto reference_kf = dynamic_cast<MonocularFrame *>(reference_keyframe.get());

  // Ensure bows are computed
  reference_kf->ComputeBow();
  ComputeBow();

  features::matching::SNNMatcher bow_matcher(0.7);
  std::vector<features::Match> matches;
  features::matching::iterators::BowIterator
      bow_it(features_.bow_container.feature_vector, reference_kf->features_.bow_container.feature_vector);

  features::matching::validators::BowMatchTrackingValidator
      validator(map_points_, reference_kf->map_points_, false, true);
  features::matching::validators::OrientationValidator
      orientation_validator(features_.keypoints, reference_kf->features_.keypoints);

  bow_matcher.MatchWithIterator(features_.descriptors,
                                reference_kf->features_.descriptors,
                                matches,
                                &bow_it,
                                {&validator},
                                &orientation_validator);

  if (matches.size() < 15) {
    return false;
  }

  for (const auto & match: matches) {
    auto map_point = reference_kf->map_points_[match.from_idx];
    map_points_[match.to_idx] = map_point;
  }
  std::unordered_set<std::size_t> inliers;
  SetPosition(*(reference_kf->GetPose()));
  OptimizePose(inliers);
  for (const auto & match: matches) {
    if (inliers.find(match.to_idx) != inliers.end()) {
      map_points_[match.to_idx]->AddObservation(this, match.to_idx);
      map_points_[match.to_idx]->Refresh();
    } else
      map_points_.erase(match.to_idx);
  }
  covisibility_connections_.Update();
  reference_kf->covisibility_connections_.Update();
  return inliers.size() > 3u;
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

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
  g2o::VertexSE3Expmap *pose = CreatePoseVertex();
  optimizer.addVertex(pose);
  size_t last_id = Identifiable::GetNextId();
  std::unordered_map<optimization::edges::SE3ProjectXYZPoseOnly *, std::size_t> edges;

  for (auto mp_id:map_points_) {
    map::MapPoint *map_point = mp_id.second;
    size_t feature_id = mp_id.first;
    if (nullptr == map_point)
      continue;
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
#ifndef NDEBUG
  { ;
    std::stringstream ss;
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif
  optimizer.initializeOptimization(0);
//  optimizer.setVerbose(true);

  for (int i = 0; i < 4; ++i) {
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
  logging::RetrieveLogger()->info("Tracking frame {}", Id());
#ifndef NDEBUG
  {
    std::stringstream ss;
    ss << pose_.R << std::endl << pose_.T << std::endl;
    logging::RetrieveLogger()->debug(ss.str());
  }
#endif

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

bool MonocularFrame::BaselineIsNotEnough(const MonocularFrame *other) const {
  TVector3D baseline = pose_.T - other->pose_.T;
  precision_t baseline_length = baseline.norm();
  precision_t frame_median_depth = other->ComputeMedianDepth();
  return baseline_length / frame_median_depth < 1e-3;
}

void MonocularFrame::ComputeMatches(const MonocularFrame *keyframe,
                                    vector<features::Match> & out_matches,
                                    geometry::Pose & out_pose) const {
  const precision_t th = 0.6f;
  features::matching::SNNMatcher matcher(th);
  features::matching::validators::BowMatchTrackingValidator
      validator(map_points_, keyframe->map_points_, false, false);

  geometry::utils::ComputeRelativeTransformation(pose_,
                                                 keyframe->pose_,
                                                 out_pose);
  features::matching::validators::BowMatchLocalMappingValidator lm_validator(features_,
                                                                             keyframe->features_,
                                                                             feature_extractor_.get(),
                                                                             keyframe->feature_extractor_.get(),
                                                                             camera_->FxInv(),
                                                                             keyframe->camera_->FxInv(),
                                                                             &pose_);
  features::matching::iterators::BowIterator
      bow_iterator(features_.bow_container.feature_vector, keyframe->features_.bow_container.feature_vector);
  features::matching::validators::OrientationValidator
      orientation_validator(features_.keypoints, keyframe->features_.keypoints);

  matcher.MatchWithIterator(features_.descriptors,
                            keyframe->features_.descriptors,
                            out_matches,
                            &bow_iterator,
                            {&validator, &lm_validator},
                            &orientation_validator);

}

void MonocularFrame::FindNewMapPoints() {
  std::unordered_set<frame::FrameBase *> neighbour_keyframes = CovisibilityGraph().GetCovisibleKeyFrames(20);
  for (frame::FrameBase *frame : neighbour_keyframes) {
    if (frame->Type() != Type())
      continue;
    auto keyframe = dynamic_cast<MonocularFrame *>(frame);
    ComputeBow();
    keyframe->ComputeBow();

    if (BaselineIsNotEnough(keyframe)) {
      logging::RetrieveLogger()->debug("Baseline between frames  {} and {} is not enough", Id(), keyframe->Id());
      continue;
    }
    std::vector<features::Match> matches;
    geometry::Pose relative_pose;
    ComputeMatches(keyframe, matches, relative_pose);
    logging::RetrieveLogger()->debug("Local mapper found {} new map-points between {} and {}",
                                     matches.size(),
                                     Id(),
                                     keyframe->Id());

    for (auto & match:matches) {
      TPoint3D pt;
      geometry::utils::Triangulate(relative_pose,
                                   keyframe->features_.undistorted_keypoints[match.from_idx],
                                   features_.undistorted_keypoints[match.to_idx],
                                   pt);
      auto mp = new map::MapPoint(keyframe->pose_.R.transpose()
                                      * (pt - keyframe->pose_.T));
      map_points_[match.to_idx] = mp;
      keyframe->map_points_[match.from_idx] = mp;
      mp->AddObservation(this, match.to_idx);
      mp->AddObservation(keyframe, match.from_idx);
      mp->Refresh();
      covisibility_connections_.Update();
      keyframe->covisibility_connections_.Update();
    }
  }

  // We will reuse neighbour_keyframes bearing in mind that the initial frame can still be there
  std::set<map::MapPoint *> map_points;
  this->ListMapPoints(neighbour_keyframes, map_points);
  std::unordered_set<FrameBase *> fixed_frames;
  this->FixedFrames(map_points, neighbour_keyframes, fixed_frames);


}

}
}  // namespace orb_slam3