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
#include <map/atlas.h>

#include <debug/debug_utils.h>

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
    auto edge = new edges::SE3ProjectXYZPoseOnly(map_point, feature_id, frame->GetMonoCamera(), map_point->GetPosition());
    edge->setId(++max_id);
    edge->setVertex(0, frame_vertex);

//    edge->setMeasurement(features.undistorted_keypoints[feature_id]);
    edge->setMeasurement(features.keypoints[feature_id].pt);
    precision_t
        information_coefficient =
        1. / frame->GetAtlas()->GetFeatureExtractor()->GetAcceptableSquareError(features.keypoints[feature_id].level);

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
  }

  logging::RetrieveLogger()->debug("Pose optimization discarded {} map_points", discarded_count);

  frame->SetStagingPosition(frame_vertex->estimate());
  frame->ApplyStaging();

}

size_t OptimizeSim3(const frame::monocular::MonocularKeyFrame * const to_frame,
                    const frame::monocular::MonocularKeyFrame * const from_frame,
                    geometry::Sim3Transformation & in_out_transformation,
                    const std::unordered_map<map::MapPoint *, size_t> & matches,
                    const std::unordered_map<map::MapPoint *, int> & predicted_levels) {

  const precision_t threshold = to_frame->GetSensorConstants()->sim3_optimization_threshold;
  g2o::SparseOptimizer optimizer;
  InitializeOptimizer<g2o::LinearSolverDense, g2o::BlockSolverX>(optimizer);

  int max_id = Sim3FillOptimizer(optimizer, to_frame, from_frame,
                                 in_out_transformation,
                                 matches,
                                 predicted_levels);

  auto trans_vertex = dynamic_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
  assert(nullptr != trans_vertex);

  in_out_transformation.print();
  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(5);
  in_out_transformation.R = trans_vertex->estimate().rotation().toRotationMatrix();
  in_out_transformation.T = trans_vertex->estimate().translation();
  in_out_transformation.s = trans_vertex->estimate().scale();
  in_out_transformation.print();



  for (int i = 1; i <= max_id; ++i) {
    auto from_mp_vertex = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(4 * i - 3));
    if (nullptr == from_mp_vertex) continue;
    // There is only one edge coming out from this vertex
    auto from_to_edge = dynamic_cast<g2o::EdgeSim3ProjectXYZ *>(*from_mp_vertex->edges().begin());
    assert(nullptr != from_to_edge);

    auto to_mp_vertex = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(4 * i - 1));
    assert(nullptr != to_mp_vertex);
    auto to_from_edge = dynamic_cast<g2o::EdgeInverseSim3ProjectXYZ *>(*to_mp_vertex->edges().begin());
    assert(nullptr != to_from_edge);

    if (from_to_edge->chi2() > threshold || to_from_edge->chi2() > threshold) {
      optimizer.removeEdge(from_to_edge);
      optimizer.removeVertex(from_mp_vertex);
      optimizer.removeEdge(to_from_edge);
      optimizer.removeVertex(to_mp_vertex);
    } else {
      from_to_edge->setRobustKernel(nullptr);
      to_from_edge->setRobustKernel(nullptr);
    }
  }

  if (optimizer.edges().size() < to_frame->GetSensorConstants()->min_number_of_edges_sim3_opt)
    return 0;

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  in_out_transformation.R = trans_vertex->estimate().rotation().toRotationMatrix();
  in_out_transformation.T = trans_vertex->estimate().translation();
  in_out_transformation.s = trans_vertex->estimate().scale();
  in_out_transformation.print();
#ifndef MULTITHREADED
  cv::imshow("projected matches", debug::DrawMapPointMatches(to_frame, from_frame, matches));
  cv::waitKey();
#endif
  return optimizer.edges().size() / 2;

}

}
}