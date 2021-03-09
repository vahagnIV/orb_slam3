//
// Created by vahagn on 09.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_UTILS_ADD_FRAMES_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_UTILS_ADD_FRAMES_H_

// === g2o ===
#include <g2o/core/sparse_optimizer.h>

// === orb_slam3 ===
#include <frame/frame_base.h>
namespace orb_slam3 {

void AddFrames(g2o::SparseOptimizer & optimizer, const std::vector<frame::FrameBase *> & frames);

}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_UTILS_ADD_FRAMES_H_
