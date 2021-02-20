//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_MAP_H_
#define ORB_SLAM3_INCLUDE_MAP_H_

// == stl =====
#include <unordered_set>

// == orb-slam3 ===
#include <frame/frame_base.h>

namespace orb_slam3 {
namespace map {
class Map {
 public:
  void AddKeyFrame(const std::shared_ptr<frame::FrameBase> & frame);

 private:
  std::unordered_set<std::shared_ptr<frame::FrameBase>> key_frames_;



};
}
}
#endif //ORB_SLAM3_INCLUDE_MAP_H_
