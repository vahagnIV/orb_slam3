//
// Created by vahagn on 17/05/2021.
//

#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sim3/sim3.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include "monocular_optimization.h"
#include "utils.h"
#include "vertices/frame_vertex.h"
#include "vertices/map_point_vertex.h"
#include "edges/se3_project_xyz_pose_only.h"
#include <logging.h>
#include <map/map_point.h>

namespace orb_slam3 {
namespace optimization {

using namespace frame::monocular;

void OptimizePose(MonocularFrame * frame) {
  BaseMonocular::MonocularMapPoints map_points = frame->GetMapPoints();

  g2o::SparseOptimizer optimizer;
  InitializeOptimizer<g2o::LinearSolverDense>(optimizer);

  geometry::Pose pose = frame->GetPosition();

  auto frame_vertex = new vertices::FrameVertex(frame);
  size_t max_id = frame->Id();
  frame_vertex->setId(max_id);
  frame_vertex->setFixed(false);
  optimizer.addVertex(frame_vertex);

  const auto & features = frame->GetFeatureHandler()->GetFeatures();

  for (auto mp_id: map_points) {
    map::MapPoint * map_point = mp_id.second;
    size_t feature_id = mp_id.first;
    if (map_point->IsBad())
      continue;
    auto edge = new edges::SE3ProjectXYZPoseOnly(map_point, feature_id, frame->GetCamera(), map_point->GetPosition());
    edge->setId(++max_id);
    edge->setVertex(0, frame_vertex);

//    edge->setMeasurement(features.undistorted_keypoints[feature_id]);
    edge->setMeasurement(features.keypoints[feature_id].pt);
    precision_t
        information_coefficient =
        1. / frame->GetFeatureExtractor()->GetAcceptableSquareError(features.keypoints[feature_id].level);

    edge->setInformation(Eigen::Matrix2d::Identity() * information_coefficient);

    auto rk = new g2o::RobustKernelHuber;
    edge->setRobustKernel(rk);
    rk->setDelta(constants::HUBER_MONO_DELTA);
    edge->setLevel(0);
    optimizer.addEdge(edge);
  }

  const int N = 4;

  size_t discarded_count = 0;
  for (int i = 0; i < N; ++i) {
    dynamic_cast<vertices::FrameVertex *>(optimizer.vertex(frame_vertex->id()))->setEstimate(pose.GetQuaternion());
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    for (auto edge_base: optimizer.edges()) {
      auto edge = dynamic_cast<edges::SE3ProjectXYZPoseOnly *>(edge_base);
      if (1 == edge->level()) { // If  the edge was not included in the optimization
        edge->computeError();
      }
      if (edge->IsValid(constants::MONO_CHI2)) {
        edge->setLevel(0);
      } else {
        if (N - 1 == i) {
          frame->EraseMapPoint(edge->GetFeatureId());
          ++discarded_count;
        } else
          edge->setLevel(1);
      }
      if (N - 2 == i)
        edge->setRobustKernel(nullptr);
    }

    // std::cout << "EDGE COUNT " << optimizer.edges().size() << std::endl;
    //std::cout << "VERTEX POSE " << i << frame_vertex->estimate() << std::endl;

  }

  logging::RetrieveLogger()->debug("Pose optimization discarded {} map_points", discarded_count);

  frame->SetPosition(frame_vertex->estimate());

}

void OptimizeSim3(const MonocularKeyFrame * const to_frame,
                  const MonocularKeyFrame * const from_frame,
                  geometry::Sim3Transformation & in_out_transformation,
                  const std::unordered_map<map::MapPoint *, size_t> & matches) {
  g2o::SparseOptimizer optimizer;
  InitializeOptimizer<g2o::LinearSolverDense, g2o::BlockSolverX>(optimizer);

  int id_counter = 0;
  g2o::Sim3 sim3(in_out_transformation.R, in_out_transformation.T, in_out_transformation.s);
  auto transformation_vertex = new g2o::VertexSim3Expmap();
  transformation_vertex->setEstimate(sim3);
  transformation_vertex->setId(id_counter);
  transformation_vertex->_focal_length1 << to_frame->GetCamera()->Fx(), to_frame->GetCamera()->Fy();
  transformation_vertex->_focal_length2 << from_frame->GetCamera()->Fx(), from_frame->GetCamera()->Fy();
  optimizer.addVertex(transformation_vertex);

  const geometry::Pose & to_pose = to_frame->GetPosition();
  const geometry::Pose & from_pose = from_frame->GetPosition();
  const features::Features & to_features = to_frame->GetFeatureHandler()->GetFeatures();
  const features::Features & from_features = from_frame->GetFeatureHandler()->GetFeatures();

  auto to_map_points = to_frame->GetMapPoints();
  std::unordered_map<size_t, map::MapPoint *> inverted_matches;
  std::transform(matches.begin(),
                 matches.end(),
                 std::inserter(inverted_matches, inverted_matches.begin()),
                 [](const std::pair<map::MapPoint *, size_t> & pair) {
                   return std::pair<size_t, map::MapPoint *>(pair.second, pair.first);
                 });
  for (auto it: to_map_points) {
    size_t feature_id = it.first;
    map::MapPoint * to_mp = it.second;
    if (to_mp->IsBad())
      continue;

    auto match_it = inverted_matches.find(feature_id);
    if (match_it != inverted_matches.end()) {
      map::MapPoint * from_mp = match_it->second;
      if (from_mp->IsBad())
        continue;

      auto to_mp_vertex = new g2o::VertexPointXYZ();
      to_mp_vertex->setId(++id_counter);
      to_mp_vertex->setEstimate(to_pose.Transform(to_mp->GetPosition()));
      to_mp_vertex->setFixed(true);

      auto from_mp_vertex = new g2o::VertexPointXYZ();
      from_mp_vertex->setId(++id_counter);
      from_mp_vertex->setEstimate(from_pose.Transform(from_mp->GetPosition()));
      from_mp_vertex->setFixed(true);
      if (from_mp->IsInKeyFrame(from_frame)) {
        auto obs = from_mp->Observation(from_frame);
        auto from_edge = new g2o::EdgeInverseSim3ProjectXYZ();
        from_edge->setId(++id_counter);
        from_edge->setMeasurement(from_features.undistorted_keypoints[obs.GetFeatureId()]);
        from_edge->setInformation(TMatrix22::Identity()
                                      / from_frame->GetFeatureExtractor()->GetAcceptableSquareError(from_features.keypoints[obs.GetFeatureId()].level));
        from_edge->setVertex(0, from_mp_vertex);
        from_edge->setVertex(1, transformation_vertex);
        optimizer.addEdge(from_edge);

      }

      auto to_edge = new g2o::EdgeSim3ProjectXYZ();
      to_edge->setId(++id_counter);
      to_edge->setMeasurement(to_features.undistorted_keypoints[feature_id]);
      to_edge->setInformation(TMatrix22::Identity()
                                  / to_frame->GetFeatureExtractor()->GetAcceptableSquareError(to_features.keypoints[feature_id].level));
      to_edge->setVertex(0, to_mp_vertex);
      to_edge->setVertex(1, transformation_vertex);
      optimizer.addEdge(to_edge);

    }
  }

}

}
}