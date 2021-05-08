//
// Created by vahagn on 08.05.21.
//

#include "optimization/bundle_adjustment.h"

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

}

}
}