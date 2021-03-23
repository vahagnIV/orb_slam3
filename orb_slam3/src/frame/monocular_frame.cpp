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
#include <features/second_nearest_neighbor_matcher.h>
#include <geometry/two_view_reconstructor.h>
#include <geometry/utils.h>
#include <features/bow_matcher.h>
#include <optimization/edges/se3_project_xyz_pose.h>
#include <optimization/edges/se3_project_xyz_pose_only.h>
#include <optimization/bundle_adjustment.h>

namespace orb_slam3 {
namespace frame {

MonocularFrame::MonocularFrame(const TImageGray8U &image, TimePoint timestamp,
                               const std::shared_ptr<features::IFeatureExtractor> &feature_extractor,
                               const std::shared_ptr<camera::MonocularCamera> &camera,
                               features::BowVocabulary *vocabulary) :
    FrameBase(timestamp),
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

bool MonocularFrame::Link(const std::shared_ptr<FrameBase> &other) {
  if (other->Type() != Type())
    return false;
  MonocularFrame *from_frame = dynamic_cast<MonocularFrame *>(other.get());
  features::SNNMatcher matcher(100,
                               0.9,
                               true);
  matcher.Match(features_, from_frame->features_, frame_link_.matches);
  if (frame_link_.matches.size() < 100)
    return false;

  geometry::TwoViewReconstructor reconstructor(5, camera_->FxInv());
  std::vector<TPoint3D> points;
  TMatrix33 rotation_matrix;
  TVector3D translation_vector;
  if (reconstructor.Reconstruct(features_.undistorted_keypoints,
                                from_frame->features_.undistorted_keypoints,
                                frame_link_.matches,
                                rotation_matrix,
                                translation_vector,
                                points,
                                frame_link_.inliers)) {
    pose_.setEstimate(geometry::Quaternion(rotation_matrix, translation_vector));
    // TODO: pass to asolute R,T
    frame_link_.other = other;

    for (size_t i = 0; i < frame_link_.matches.size(); ++i) {
      if (!frame_link_.inliers[i])
        continue;
      const features::Match &match = frame_link_.matches[i];

      if (other->MapPoint(match.from_idx)) {
        // TODO: do the contistency check
      } else {

        auto map_point = new map::MapPoint(points[i]);
        map_points_[match.to_idx] = map_point;
        other->MapPoint(match.from_idx) = map_points_[match.to_idx];
        map_point->AddObservation(this, frame_link_.matches[i].to_idx);
        map_point->AddObservation(from_frame, frame_link_.matches[i].from_idx);
      }
    }
    from_frame->CovisibilityGraph().Update();
    this->CovisibilityGraph().Update();
    std::cout << "Frame " << Id() << " " << pose_.estimate().rotation().toRotationMatrix() << std::endl
              << pose_.estimate().translation() << std::endl;
    optimization::BundleAdjustment({this, from_frame}, 20);
    // TODO: normalize T
    std::cout << "Frame " << Id() << " " << pose_.estimate().rotation().toRotationMatrix() << std::endl
              << pose_.estimate().translation() << std::endl;
    return true;
  }

  return false;
}

FrameType MonocularFrame::Type() const {
  return MONOCULAR;
}

void MonocularFrame::AppendDescriptorsToList(size_t feature_id,
                                             std::vector<features::DescriptorType> &out_descriptor_ptr) const {

  out_descriptor_ptr.emplace_back(features_.descriptors.row(feature_id));

}

void MonocularFrame::AppendToOptimizerBA(g2o::SparseOptimizer &optimizer, size_t &next_id) {
  g2o::VertexSE3Expmap *pose = CreatePoseVertex();
  optimizer.addVertex(pose);
  for (auto &mp_id:map_points_) {
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
    HomogenousPoint measurement;
    camera_->UnprojectPoint(features_.keypoints[feature_id].pt, measurement);
    edge->setMeasurement(Eigen::Map<Eigen::Matrix<double, 2, 1>>(measurement.data()));
    optimizer.addEdge(edge);
  }
}

void MonocularFrame::CollectFromOptimizerBA(g2o::SparseOptimizer &optimizer) {
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *> (optimizer.vertex(Id()));
  pose_.setEstimate(pose->estimate());
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

TPoint3D MonocularFrame::GetNormal(const TPoint3D &point) const {
  TPoint3D normal = pose_.estimate().translation() - point;
  normal.normalize();
  return normal;
}

bool MonocularFrame::TrackWithReferenceKeyFrame(const std::shared_ptr<FrameBase> &reference_keyframe) {
  if (reference_keyframe->Type() != Type())
    return false;
  auto reference_kf = dynamic_cast<MonocularFrame *>(reference_keyframe.get());

  // Ensure bows are computed
  reference_kf->ComputeBow();
  ComputeBow();

  features::BowMatcher bow_matcher(0.7);
  std::vector<features::Match> matches;
  std::unordered_set<std::size_t> inliers, rf_inliers;
  std::transform(map_points_.begin(),
                 map_points_.end(),
                 std::inserter(inliers, inliers.begin()),
                 [](decltype(map_points_)::value_type &it) { return it.first; });
  std::transform(reference_kf->map_points_.begin(),
                 reference_kf->map_points_.end(),
                 std::inserter(rf_inliers, rf_inliers.begin()),
                 [](decltype(map_points_)::value_type &it) { return it.first; });
  bow_matcher.Match(features_,
                    reference_kf->features_,
                    inliers,
                    rf_inliers,
                    matches);
  if (matches.size() < 15)
    return false;

  for (const auto &match: matches) {
    auto map_point = reference_kf->map_points_[match.to_idx];
    map_points_[match.from_idx] = map_point;
  }
  SetPosition(*(reference_kf->GetPose()));
  OptimizePose(inliers);
  for (const auto &match: matches) {
    if (inliers.find(match.from_idx) != inliers.end()) {
      map_points_[match.from_idx]->AddObservation(this, match.from_idx);
      map_points_[match.from_idx]->Refresh();
    } else
      map_points_.erase(match.from_idx);
  }
  return inliers.size() > 3u;
}

void MonocularFrame::ComputeBow() {
  features_.ComputeBow();
}

void MonocularFrame::OptimizePose(std::unordered_set<std::size_t> &out_inliers) {
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
//  std::cout << "Frame " << Id() << std::endl;
//  std::cout << pose_.estimate().rotation().toRotationMatrix() << std::endl;
//  std::cout << pose_.estimate().translation() << std::endl;
  optimizer.initializeOptimization(0);
//  optimizer.setVerbose(true);

  for (int i = 0; i < 4; ++i) {
    pose->setEstimate(pose_.estimate());
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

  SetPosition(*pose);
  std::cout << "Tracking Frame " << Id() << std::endl;
  std::cout << pose_.estimate().rotation().toRotationMatrix() << std::endl;
  std::cout << pose_.estimate().translation() << std::endl;

}

precision_t MonocularFrame::ComputeMedianDepth() const {
  const g2o::SE3Quat &pose_quat = pose_.estimate();
  std::vector<precision_t> depths(map_points_.size());
  std::transform(map_points_.begin(),
                 map_points_.end(),
                 depths.begin(),
                 [&pose_quat](const std::pair<size_t, map::MapPoint *> &mp_id) -> precision_t {
                   return pose_quat.map(mp_id.second->GetPosition())[2];
                 });
  return depths[(depths.size() - 1) / 2];
}

bool MonocularFrame::BaselineIsNotEnough(const MonocularFrame *other) const {
  TVector3D baseline = pose_.estimate().translation() - other->pose_.estimate().translation();
  precision_t baseline_length = baseline.norm();
  precision_t frame_median_depth = other->ComputeMedianDepth();
  return baseline_length / frame_median_depth < 1e-2;
}

TMatrix33 MonocularFrame::ComputeRelativeFundamentalMatrix(const MonocularFrame *other) const {
  TMatrix33 R;
  TVector3D T;
  geometry::utils::ComputeRelativeTransformation(pose_.estimate().rotation().toRotationMatrix(),
                                                 pose_.estimate().translation(),
                                                 other->pose_.estimate().rotation().toRotationMatrix(),
                                                 other->pose_.estimate().translation(),
                                                 R,
                                                 T);

  return geometry::FundamentalMatrixEstimator::FromEuclideanTransformations(R, T);
}

void MonocularFrame::FindNewMapPoints() {
  std::vector<frame::FrameBase *> neighbour_keyframes = CovisibilityGraph().GetCovisibleKeyFrames(20);
  for (frame::FrameBase *frame : neighbour_keyframes) {
    if (frame->Type() != Type())
      continue;
    auto keyframe = dynamic_cast<MonocularFrame *>(frame);
    if (BaselineIsNotEnough(keyframe))
      continue;
    TMatrix33 F12 = ComputeRelativeFundamentalMatrix(keyframe);
    precision_t th = 0.6f;
    features::SNNMatcher matcher(100, th, false);
  }

}

}
}  // namespace orb_slam3