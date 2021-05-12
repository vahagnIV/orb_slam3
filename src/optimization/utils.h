//
// Created by vahagn on 12/05/2021.
//

#ifndef ORB_SLAM3_SRC_OPTIMIZATION_UTILS_H_
#define ORB_SLAM3_SRC_OPTIMIZATION_UTILS_H_

// === g2o ===
#include <g2o/core/sparse_optimizer.h>

// === orb_slam3 ===
#include <frame/key_frame.h>
#include "vertices/frame_vertex.h"
#include "vertices/map_point_vertex.h"
namespace orb_slam3 {
namespace optimization {

void InitializeOptimizer(g2o::SparseOptimizer & optimizer);

size_t FillKeyFrameVertices(const std::unordered_set<frame::KeyFrame *> & key_frames,
                            g2o::SparseOptimizer & inout_optimizer,
                            std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & out_frame_map);

void FillMpVertices( const std::unordered_set<map::MapPoint *> & map_points,
                     g2o::SparseOptimizer & inout_optimizer,
                     const std::unordered_map<frame::KeyFrame *, vertices::FrameVertex *> & frame_map,
                     std::unordered_map<map::MapPoint *, vertices::MapPointVertex *> & out_mp_map,
                     bool robust,
                     size_t & inout_id);

}
}
#endif //ORB_SLAM3_SRC_OPTIMIZATION_UTILS_H_
