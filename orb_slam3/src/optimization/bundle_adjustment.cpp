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

//  AddFrames(optimizer, key_frames);
  size_t last_id = Identifiable::GetNextId();
  size_t init_frame_id = std::numeric_limits<size_t>::max();
  for (auto frame: key_frames) {
    init_frame_id = std::min(init_frame_id, frame->Id());
    frame->AddToOptimizer(optimizer, last_id);
  }
  optimizer.vertex(init_frame_id)->setFixed(true);
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(nIterations);
  for(auto frame: key_frames){
    for (auto mp: frame->MapPoints()) {
      if(nullptr == mp)
        continue;
      auto mp_as_vertex = mp->operator g2o::VertexPointXYZ *();
      auto  optimized = dynamic_cast<g2o::VertexPointXYZ* >(optimizer.vertex(mp_as_vertex->id()));
      mp_as_vertex->setEstimate(optimized->estimate());
    }
  }
  std::cout << "asd" << std::endl;
}

}
}