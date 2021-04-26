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

CovisibilityGraphNode::CovisibilityGraphNode(const frame::FrameBase * frame) : frame_(frame) {

}

void CovisibilityGraphNode::Update() {

  std::unordered_set<map::MapPoint *> local_map_points_;
  std::unordered_map<FrameBase *, unsigned> connected_frame_weights;
  frame_->ListMapPoints(local_map_points_);
  for (auto map_point:local_map_points_) {
    for (auto obs: map_point->Observations()) {
      ++connected_frame_weights[obs.first];
    }
  }
  std::vector<std::pair<unsigned, FrameBase *>> weight_frame_pairs;

  weight_frame_pairs.reserve(connected_frame_weights.size());
  for (auto & fp: connected_frame_weights)
    weight_frame_pairs.emplace_back(fp.second, fp.first);

  std::sort(weight_frame_pairs.begin(), weight_frame_pairs.end());

  std::unique_lock<std::mutex> lock_guard(mutex_);
  sorted_connected_frames_.clear();
  sorted_weights_.clear();
  sorted_weights_.reserve(weight_frame_pairs.size());
  sorted_connected_frames_.reserve(weight_frame_pairs.size());
  for (decltype(weight_frame_pairs)::reverse_iterator p = weight_frame_pairs.rbegin(); p != weight_frame_pairs.rend();
       ++p) {
    sorted_weights_.push_back(p->first);
    sorted_connected_frames_.push_back(p->second);
  }
}

std::unordered_set<FrameBase *> CovisibilityGraphNode::GetCovisibleKeyFrames(unsigned int count) {
  std::lock_guard<std::mutex> m(mutex_);
  std::unordered_set<FrameBase *> result;
  std::copy_if(sorted_connected_frames_.begin(),
               sorted_connected_frames_.end(),
               std::inserter(result, result.begin()),
               [](const FrameBase * frame) { return frame->IsValid(); });
  return result;

}

}
}