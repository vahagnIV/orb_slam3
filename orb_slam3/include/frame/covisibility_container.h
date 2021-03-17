//
// Created by vahagn on 16/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_CONTAINER_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_CONTAINER_H_

// === stl ===
#include <mutex>
#include <vector>
#include <unordered_map>
#include <map>

namespace orb_slam3 {
namespace frame {
class FrameBase;

struct CovisibilityContainer {

  void Update();
  void AddConnection(FrameBase * frame);
  void RemoveConnection(FrameBase * frame);

  std::unordered_map<FrameBase *, unsigned> connected_frame_weights;
  std::vector<FrameBase *> sorted_connected_frames;
  std::vector<size_t> sorted_weights;



};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_CONTAINER_H_
