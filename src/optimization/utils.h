//
// Created by vahagn on 12/05/2021.
//

#ifndef ORB_SLAM3_SRC_OPTIMIZATION_UTILS_H_
#define ORB_SLAM3_SRC_OPTIMIZATION_UTILS_H_

// === g2o ===
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

// === orb_slam3 ===
#include <frame/key_frame.h>
#include "vertices/frame_vertex.h"
#include "vertices/map_point_vertex.h"
namespace orb_slam3 {
namespace optimization {

template<template <typename> class T, typename SolverType = g2o::BlockSolver_6_3>
void InitializeOptimizer(g2o::SparseOptimizer & optimizer){
  std::unique_ptr<typename SolverType::LinearSolverType>  linearSolver(new T<typename SolverType::PoseMatrixType>());
  std::unique_ptr<SolverType> solver_ptr(new SolverType(std::move(linearSolver)));
  auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
  //optimizer.setVerbose(true);
}

size_t FillKeyFrameVertices(const std::unordered_set<frame::KeyFrame *> & key_frames,
                            g2o::SparseOptimizer & inout_optimizer,
                            std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & out_frame_map,
                            bool fixed = false);

void FillMpVertices( const std::unordered_set<map::MapPoint *> & map_points,
                     g2o::SparseOptimizer & inout_optimizer,
                     const std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & frame_map,
                     std::unordered_map<map::MapPoint *, vertices::MapPointVertex *> & out_mp_map,
                     bool robust,
                     size_t & inout_id);

g2o::VertexPointXYZ * CreateVertex(const map::MapPoint * map_point, const geometry::Pose & pose);

}
}
#endif //ORB_SLAM3_SRC_OPTIMIZATION_UTILS_H_
