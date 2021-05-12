//
// Created by vahagn on 08.05.21.
//

#include "bundle_adjustment.h"
#include <frame/observation.h>
#include <map/map_point.h>
#include "optimization/vertices/map_point_vertex.h"
#include "optimization/vertices/frame_vertex.h"
#include "optimization/edges/ba_binary_edge.h"
#include "optimization/edges/ba_unary_edge.h"
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
  InitializeOptimizer(optimizer);
  optimizer.setVerbose(true);

  std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> frame_map;
  std::unordered_map<map::MapPoint *, vertices::MapPointVertex *> mp_map;

  size_t max_kf_id = FillKeyFrameVertices(key_frames, optimizer, frame_map);
  FillMpVertices(map_points, optimizer, frame_map, mp_map, robust, max_kf_id);

  optimizer.initializeOptimization();
  optimizer.setForceStopFlag(stop_flag);
  optimizer.optimize(number_of_iterations);


  // Collect frame positions
  for (auto frame_vertex: frame_map) {
    if (nullptr == loop_kf || loop_kf->IsInitial())
      frame_vertex.first->SetPosition(frame_vertex.second->estimate());
    else {
      // TODO: Implement for loop closing
      std::runtime_error("This is not implemented yet");
    }
  }

  // Collect Map Point positions
  for (auto mp_vertex: mp_map) {
    if (nullptr == loop_kf || loop_kf->IsInitial()) {
      mp_vertex.first->SetPosition(mp_vertex.second->estimate());
      mp_vertex.first->UpdateNormalAndDepth();
    } else {
      std::runtime_error("BA not implemented for Loop closing");
    }
  }
}

}
}