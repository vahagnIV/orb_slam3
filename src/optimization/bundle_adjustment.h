//
// Created by vahagn on 08.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_BUNDLE_ADJUSTMENT_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_BUNDLE_ADJUSTMENT_H_

// == orb_slam3 ===

#include <frame/key_frame.h>

namespace orb_slam3 {
namespace optimization {

void BundleAdjustment(std::unordered_set<frame::KeyFrame *> & key_frames,
                      std::unordered_set<map::MapPoint *> & map_points,
                      unsigned number_of_iterations = 5,
                      bool * stop_flag = nullptr,
                      frame::KeyFrame * loop_kf = nullptr,
                      bool robust = true);

void LocalBundleAdjustment(std::unordered_set<frame::KeyFrame *> & keyframes,
                           std::unordered_set<frame::KeyFrame *> & fixed_keyframes,
                           frame::BaseFrame::MapPointSet & local_map_points,
                           std::vector<std::pair<map::MapPoint *, frame::KeyFrame *>> & out_observations_to_delete,
                           bool * stop_flag);


}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_BUNDLE_ADJUSTMENT_H_
