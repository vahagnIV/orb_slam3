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
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <frame/monocular/monocular_key_frame.h>
#include "vertices/frame_vertex.h"
#include "vertices/map_point_vertex.h"
namespace orb_slam3 {
namespace optimization {

template<template<typename> class T, typename SolverType = g2o::BlockSolver_6_3>
void InitializeOptimizer(g2o::SparseOptimizer & optimizer) {
  std::unique_ptr<typename SolverType::LinearSolverType> linearSolver(new T<typename SolverType::PoseMatrixType>());
  std::unique_ptr<SolverType> solver_ptr(new SolverType(std::move(linearSolver)));
  auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);
  //optimizer.setVerbose(true);
}

size_t FillKeyFrameVertices(const std::unordered_set<frame::KeyFrame *> & key_frames,
                            g2o::SparseOptimizer & inout_optimizer,
                            std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & out_frame_map,
                            bool fixed = false);

void FillMpVertices(const std::unordered_set<map::MapPoint *> & map_points,
                    g2o::SparseOptimizer & inout_optimizer,
                    const std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & frame_map,
                    std::unordered_map<map::MapPoint *, vertices::MapPointVertex *> & out_mp_map,
                    bool robust,
                    size_t & inout_id);

g2o::VertexPointXYZ * CreateVertex(const map::MapPoint * map_point, const geometry::Pose & pose);

template<typename TEdge>
TEdge * CreateEdge(const features::Features & features,
                   const features::IFeatureExtractor * feature_extractor,
                   const TPoint2D & measurement,
                   int level,
                   precision_t huber_delta) {
  auto edge = new TEdge();
  edge->setMeasurement(measurement);
  edge->setInformation(TMatrix22::Identity()
                           / feature_extractor->GetAcceptableSquareError(level));
  auto * rk = new g2o::RobustKernelHuber;
  edge->setRobustKernel(rk);
  static const precision_t delta = huber_delta;
  rk->setDelta(delta);
  return edge;
}

g2o::VertexSim3Expmap * CreateSim3Vertex(const geometry::Sim3Transformation & initial_guess,
                                         const camera::MonocularCamera * to_camera,
                                         const camera::MonocularCamera * from_camera);

int Sim3FillOptimizer(g2o::SparseOptimizer & optimizer,
                      const frame::monocular::MonocularKeyFrame * const to_frame,
                      const frame::monocular::MonocularKeyFrame * const from_frame,
                      const geometry::Sim3Transformation & in_out_transformation,
                      const std::unordered_map<map::MapPoint *, size_t> & matches,
                      const std::unordered_map<map::MapPoint *, int> & predicted_levels);

}
}
#endif //ORB_SLAM3_SRC_OPTIMIZATION_UTILS_H_
