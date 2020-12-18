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
  void SetLastFrame(FrameBase *frame) noexcept;
  const FrameBase *GetLastFrame() const noexcept { return last_frame_; }
  FrameBase *GetLastFrame() noexcept { return last_frame_; }
  void AcceptLastFrame() noexcept;
  ~Map();

 private:
  std::vector<FrameBase *> key_frames_;
  FrameBase *last_frame_;
};

}
#endif //ORB_SLAM3_INCLUDE_MAP_H_
