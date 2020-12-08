//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_MAP_H_
#define ORB_SLAM3_INCLUDE_MAP_H_
#include <vector>
#include <map_point.h>
#include <frame_base.h>
namespace nvision {

class Map {
 public:
  size_t FrameCount() const noexcept { return key_frames_.size(); }
  void AddKeyFrame(const std::shared_ptr<FrameBase> &frame);

 private:
  std::vector<std::shared_ptr<FrameBase>> key_frames_;
};

}
#endif //ORB_SLAM3_INCLUDE_MAP_H_
