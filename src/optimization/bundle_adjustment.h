//
// Created by vahagn on 08.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_BUNDLE_ADJUSTMENT_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_BUNDLE_ADJUSTMENT_H_

// == orb_slam3 ===

#include <frame/base_frame.h>
#include <frame/monocular/monocular_frame.h>

namespace orb_slam3 {
namespace optimization {




void BundleAdjustment(std::unordered_set<frame::KeyFrame *> & key_frames,
                      std::unordered_set<map::MapPoint *> & map_points,
                      unsigned number_of_iterations = 5,
                      bool * stop_flag = nullptr,
                      frame::KeyFrame * loop_kf = nullptr,
                      bool robust = true );

//void BundleAdjustment(g2o::SparseOptimizer & optimizer,
//                      std::unordered_set<frame::KeyFrame *> & fixed_key_frames,
//                      std::unordered_set<map::MapPoint *> & map_points,
//                      unsigned number_of_iterations);

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_BUNDLE_ADJUSTMENT_H_
