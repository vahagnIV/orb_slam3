//
// Created by vahagn on 16.03.21.
//
// === stl ===
#include <algorithm>

// === orb-slam3 ===
#include <frame/covisibility_graph_node.h>
#include <frame/frame_base.h>

namespace orb_slam3 {
namespace frame {

void CovisibilityGraphNode::AddConnection(FrameBase *frame) {
  ++connected_frame_weights_[frame];
}

void CovisibilityGraphNode::RemoveConnection(FrameBase *frame) {
  decltype(connected_frame_weights_)::iterator it = connected_frame_weights_.find(frame);
  if (it == connected_frame_weights_.end())
    return;
  --it->second;
  if (it->second == 0)
    connected_frame_weights_.erase(it);
}

void CovisibilityGraphNode::Update() {

  std::vector<std::pair<unsigned, FrameBase *>> weight_frame_pairs;
  weight_frame_pairs.reserve(connected_frame_weights_.size());
  for (auto &fp: connected_frame_weights_)
    weight_frame_pairs.emplace_back(fp.second, fp.first);

  std::sort(weight_frame_pairs.begin(), weight_frame_pairs.end());

  std::unique_lock<std::mutex> lock_guard(mutex_);
  sorted_connected_frames_.clear();
  sorted_weights_.clear();
  sorted_weights_.reserve(weight_frame_pairs.size());
  sorted_connected_frames_.reserve(weight_frame_pairs.size());
  for (auto &p: weight_frame_pairs) {
    sorted_weights_.push_back(p.first);
    sorted_connected_frames_.push_back(p.second);
  }
}

std::vector<FrameBase *> CovisibilityGraphNode::GetCovisibleKeyFrames(unsigned int count) {
  std::lock_guard<std::mutex> m(mutex_);
  return std::vector<FrameBase *>(sorted_connected_frames_.begin(),
                                  count > sorted_connected_frames_.size() ? sorted_connected_frames_.end() :
                                  sorted_connected_frames_.begin() + count);

}

}
}