//
// Created by vahagn on 2/20/21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_LINK_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_LINK_H_

// === stl ===
#include <vector>
#include <memory>

// === orb-slam3 ===
#include <features/match.h>

namespace orb_slam3 {
namespace frame {

class FrameBase;
// TODO: Remove this class
struct FrameLink {
  std::vector<features::Match> matches;
  std::vector<bool> inliers;
  std::shared_ptr<FrameBase> other;
};

}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_FRAME_LINK_H_
