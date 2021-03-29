//
// Created by vahagn on 16/03/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_GRAPH_NODE_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_GRAPH_NODE_H_

// === stl ===
#include <mutex>
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <map>

namespace orb_slam3 {
namespace frame {
class FrameBase;

struct CovisibilityGraphNode {

  void Update();
  void AddConnection(FrameBase * frame);
  void RemoveConnection(FrameBase * frame);
  std::unordered_set<FrameBase *> GetCovisibleKeyFrames(unsigned count);

 private:
  std::unordered_map<FrameBase *, unsigned> connected_frame_weights_;
  std::vector<FrameBase *> sorted_connected_frames_;
  std::vector<size_t> sorted_weights_;
  std::mutex mutex_;



};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_GRAPH_NODE_H_
