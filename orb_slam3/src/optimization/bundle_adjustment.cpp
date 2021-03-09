//
// Created by vahagn on 08.03.21.
//


#include "optimization/bundle_adjustment.h"

// === g2o ===
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

// === orb_slam3 ===
#include <optimization/utils/add_frames.h>
#include <optimization/edges/se3_project_xyz_pose.h>

namespace orb_slam3 {
namespace optimization {

void BundleAdjustment(const std::vector<frame::FrameBase *> &key_frames,
                      const std::vector<map::MapPoint *> &map_points,
                      int nIterations) {
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>
      linearSolver(new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());

  std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));

  g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
  AddFrames(optimizer, key_frames);
  size_t last_id = Identifiable::GetNextId();
  for (auto map_point: map_points) {

    g2o::VertexPointXYZ *mp_as_vertex = map_point->operator g2o::VertexPointXYZ *();
    optimizer.addVertex(mp_as_vertex);
    for (auto observation: map_point->Observations()) {
      auto pose_vertex = optimizer.vertex(observation.first->Id());
      auto edge = new edges::SE3ProjectXYZPose(observation.first->CameraPtr());
      edge->setVertex(0, pose_vertex);
      edge->setVertex(1, mp_as_vertex);
      edge->setMeasurement(observation.first->)
      edge->setId(++last_id);
      optimizer.addEdge(edge);
    }
  }
  optimizer.optimize(nIterations);
//  optimizer.setVerbose(false);

}

}
}