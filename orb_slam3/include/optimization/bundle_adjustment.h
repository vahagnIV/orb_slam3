//
// Created by vahagn on 08.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_BUNDLE_ADJUSTMENT_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_BUNDLE_ADJUSTMENT_H_

#include <frame/frame_base.h>

namespace orb_slam3 {
namespace optimization {

void BundleAdjustment(const std::vector<frame::FrameBase *> &key_frames,
                      const std::vector<map::MapPoint *> &map_points,
                      int nIterations);

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_BUNDLE_ADJUSTMENT_H_
