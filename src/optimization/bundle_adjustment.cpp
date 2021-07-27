//
// Created by vahagn on 08.05.21.
//
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "bundle_adjustment.h"
#include <map/map_point.h>
#include <logging.h>
#include "optimization/vertices/map_point_vertex.h"
#include "optimization/vertices/frame_vertex.h"
#include "utils.h"

namespace orb_slam3 {
namespace optimization {

void BundleAdjustment(std::unordered_set<frame::KeyFrame *> & key_frames,
                      std::unordered_set<map::MapPoint *> & map_points,
                      unsigned number_of_iterations,
                      bool * stop_flag,
                      frame::KeyFrame * loop_kf,
                      bool robust) {

  g2o::SparseOptimizer optimizer;
  InitializeOptimizer<g2o::LinearSolverEigen>(optimizer);

  std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> frame_map;
  std::unordered_map<map::MapPoint *, vertices::MapPointVertex *> mp_map;

  size_t max_kf_id = FillKeyFrameVertices(key_frames, optimizer, frame_map);
  FillMpVertices(map_points, optimizer, frame_map, mp_map, robust, max_kf_id);

  optimizer.initializeOptimization();
  optimizer.setForceStopFlag(stop_flag);
  optimizer.optimize(number_of_iterations);


  // Collect frame positions
  for (auto frame_vertex: frame_map) {
    if (nullptr == loop_kf || loop_kf->IsInitial()) {
      frame_vertex.first->SetStagingPosition(frame_vertex.second->estimate());
      frame_vertex.first->ApplyStaging();
    } else {
      // TODO: Implement for loop closing
      std::runtime_error("This is not implemented yet");
    }
  }

  // Collect Map Point positions
  for (auto mp_vertex: mp_map) {
    if (nullptr == loop_kf || loop_kf->IsInitial()) {
      mp_vertex.first->SetPosition(mp_vertex.second->estimate());
      mp_vertex.first->Refresh((*key_frames.begin())->GetFeatureExtractor());
    } else {
      std::runtime_error("BA not implemented for Loop closing");
    }
  }
}

void LocalBundleAdjustment(std::unordered_set<frame::KeyFrame *> & keyframes,
                           std::unordered_set<frame::KeyFrame *> & fixed_keyframes,
                           frame::BaseFrame::MapPointSet & local_map_points,
                           bool * stop_flag) {
  g2o::SparseOptimizer optimizer;
  InitializeOptimizer<g2o::LinearSolverEigen>(optimizer);

  std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> frame_map;
  std::unordered_map<map::MapPoint *, vertices::MapPointVertex *> mp_map;
  size_t max_kf_id = FillKeyFrameVertices(keyframes, optimizer, frame_map);
  max_kf_id = std::max(max_kf_id, FillKeyFrameVertices(fixed_keyframes, optimizer, frame_map, true));
  FillMpVertices(local_map_points, optimizer, frame_map, mp_map, true, max_kf_id);

  optimizer.initializeOptimization();
  optimizer.setForceStopFlag(stop_flag);
  optimizer.optimize(5);
  if (!stop_flag || !*stop_flag)
    optimizer.optimize(10);

  size_t edge_count = optimizer.edges().size();
  std::vector<std::pair<map::MapPoint *, frame::KeyFrame *>> observations_to_delete;
  observations_to_delete.reserve(optimizer.edges().size());

  for (auto mp_vertex: mp_map) {
    optimization::vertices::MapPointVertex * vertex = mp_vertex.second;
    for (auto edge_base: vertex->edges()) {
      auto edge = dynamic_cast<optimization::edges::BABinaryEdge *>(edge_base);
      if (!edge->IsValid()) {
        auto key_frame =
            dynamic_cast<frame::KeyFrame *>(dynamic_cast<vertices::FrameVertex *>(edge->vertex(0))->GetFrame());
        observations_to_delete.emplace_back(mp_vertex.first, key_frame);
      }
    }
  }
  if (observations_to_delete.size() > edge_count / 2) {
    logging::RetrieveLogger()->warn("Local BA is aborted.  {} out {} edges are invalid",
                                    observations_to_delete.size(),
                                    edge_count);
    return;
  } else {
    logging::RetrieveLogger()->info("Local BA: removing {} edges",
                                    observations_to_delete.size());
  }

  for (auto to_delete: observations_to_delete) {
    if (!to_delete.first->IsBad())
      to_delete.second->EraseMapPoint(to_delete.first);
  }

  for (auto mp_vertex: mp_map) {
    if (!mp_vertex.first->IsBad())
      mp_vertex.first->SetPosition(mp_vertex.second->estimate());
  }
  for (auto frame_vertex: frame_map) {
    if (!frame_vertex.second->fixed()) {
      frame_vertex.first->SetStagingPosition(frame_vertex.second->estimate());
      frame_vertex.first->ApplyStaging();
    }
  }

}

}
}
