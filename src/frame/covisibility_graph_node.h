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
class KeyFrame;

struct CovisibilityGraphNode {

 public:
  CovisibilityGraphNode(const frame::KeyFrame * frame);

  void Update();
  std::unordered_set<KeyFrame *> GetCovisibleKeyFrames(unsigned count);
 private:
  const frame::KeyFrame * frame_;

  std::vector<KeyFrame *> sorted_connected_frames_;
  std::vector<size_t> sorted_weights_;
  std::mutex mutex_;



};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_FRAME_COVISIBILITY_GRAPH_NODE_H_
