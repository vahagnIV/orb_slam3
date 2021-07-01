//
// Created by vahagn on 17/05/2021.
//

#include <g2o/solvers/dense/linear_solver_dense.h>

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
  InitializeOptimizer<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>(optimizer);

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

}
}