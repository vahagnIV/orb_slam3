//
// Created by vahagn on 08.05.21.
//

#include "bundle_adjustment.h"
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
                      unordered_set<frame::KeyFrame *> & fixed_key_frames,
                      unordered_set<map::MapPoint *> & map_points,
                      unsigned int number_of_iterations) {

  InitializeOptimizer(optimizer);
  std::unordered_map<frame::KeyFrame *, int> frame_map;
  std::unordered_map<map::MapPoint *, int> mp_map;
  int id = 0;

  for (auto map_point: map_points) {
    auto mp_vertex = new vertices::MapPointVertex(map_point);
    mp_vertex->setId(id++);
    mp_map[map_point] = mp_vertex->id();
    optimizer.addVertex(mp_vertex);
    vertices::FrameVertex * frame_vertex;
    for (auto observation: map_point->Observations()) {
      auto it = frame_map.find(observation.first);
      if (it == frame_map.end()) {
        frame_vertex = new vertices::FrameVertex(observation.first);
        frame_vertex->setId(id++);
        frame_vertex->setFixed(fixed_key_frames.find(frame_vertex->GetFrame()) != fixed_key_frames.end());
        optimizer.addVertex(frame_vertex);
        frame_map[observation.first] = frame_vertex->id();
      } else
        frame_vertex = dynamic_cast<vertices::FrameVertex *>(optimizer.vertex(it->second));
      auto edge = observation.second->CreateBinaryEdge();
      edge->setVertex(0, frame_vertex);
      edge->setVertex(1, mp_vertex);
      edge->setId(id++);
      edge->setRobustKernel(observation.second->CreateRobustKernel());
      optimizer.addEdge(edge);
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(number_of_iterations);
}

}
}