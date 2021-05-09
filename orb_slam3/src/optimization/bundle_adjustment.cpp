//
// Created by vahagn on 08.05.21.
//

#include "optimization/bundle_adjustment.h"
#include "optimization/vertices/map_point_vertex.h"
#include "optimization/vertices/frame_vertex.h"

// === g2o ===
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


namespace orb_slam3 {
namespace optimization {

void InitializeOptimizer(g2o::SparseOptimizer & optimizer) {
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>
      linearSolver(new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());

  std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));

  auto * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
}

void BundleAdjustment(g2o::SparseOptimizer & optimizer,
                      unordered_set<frame::FrameBase *> & fixed_frames,
                      unordered_set<map::MapPoint *> & map_points,
                      unsigned int number_of_iterations) {
  InitializeOptimizer(optimizer);
  for (auto map_point: map_points) {
    auto mp_vertex = new vertices::MapPointVertex(map_point);
    optimizer.addVertex(mp_vertex);
    vertices::FrameVertex * frame_vertex;
    for (auto observation: map_point->Observations()) {
      if (nullptr == optimizer.vertex(observation.first->Id())) {
        frame_vertex = new vertices::FrameVertex(observation.first);
        frame_vertex->setFixed(fixed_frames.find(frame_vertex->GetFrame()) != fixed_frames.end());
        optimizer.addVertex(frame_vertex);
      } else
        frame_vertex = dynamic_cast<vertices::FrameVertex *>(optimizer.vertex(observation.first->Id()));
      auto edge = observation.second->CreateBinaryEdge();
      edge->setVertex(0, frame_vertex);
      edge->setVertex(1, mp_vertex);

      edge->setRobustKernel(observation.second->CreateRobustKernel());
      optimizer.addEdge(edge);
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(number_of_iterations);
}

}
}