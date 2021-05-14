//
// Created by vahagn on 12/05/2021.
//

#include "utils.h"

// === g2o ===
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <map/map_point.h>

namespace orb_slam3 {
namespace optimization {

void InitializeOptimizer(g2o::SparseOptimizer & optimizer) {
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>
      linearSolver(new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());

  std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));

  auto * solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
}

size_t FillKeyFrameVertices(const std::unordered_set<frame::KeyFrame *> & key_frames,
                            g2o::SparseOptimizer & inout_optimizer,
                            std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & out_frame_map) {
  size_t max_id = 0;
  for (auto key_frame: key_frames) {
    if (key_frame->IsBad())
      continue;
    auto vertex = new vertices::FrameVertex(key_frame);
    vertex->setId(key_frame->Id());
    max_id = std::max(max_id, static_cast<size_t>(vertex->id()));
    vertex->setFixed(key_frame->IsInitial());
    out_frame_map[key_frame] = vertex;
    inout_optimizer.addVertex(vertex);
  }
  return max_id;
}

void FillMpVertices(const unordered_set<map::MapPoint *> & map_points,
                    g2o::SparseOptimizer & inout_optimizer,
                    const std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & frame_map,
                    unordered_map<map::MapPoint *, vertices::MapPointVertex *> & out_mp_map,
                    bool robust,
                    size_t & inout_id) {
  for (auto map_point: map_points) {
    if (map_point->IsBad())
      continue;

    auto mp_vertex = new vertices::MapPointVertex(map_point);

    mp_vertex->setId(++inout_id);
    inout_optimizer.addVertex(mp_vertex);
    unsigned number_of_edges = 0;
    for (auto observation: map_point->Observations()) {
      auto it = frame_map.find(observation.first);
      if (it == frame_map.end())
        continue;

      vertices::FrameVertex * frame_vertex = dynamic_cast<vertices::FrameVertex *>(it->second);
      optimization::edges::BABinaryEdge *edge = observation.second.CreateBinaryEdge();
      edge->setVertex(0, frame_vertex);
      edge->setVertex(1, mp_vertex);
      edge->setId(++inout_id);
      if (robust)
        edge->setRobustKernel(observation.second.CreateRobustKernel());
      inout_optimizer.addEdge(edge);
      ++number_of_edges;
    }
    if (0 == number_of_edges)
      inout_optimizer.removeVertex(mp_vertex);
    else
      out_mp_map[map_point] = mp_vertex;
  }

}

}
}