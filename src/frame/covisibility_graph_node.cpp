//
// Created by vahagn on 16.03.21.
//
// === stl ===
#include <algorithm>

// === orb-slam3 ===
#include "covisibility_graph_node.h"
#include "key_frame.h"
#include <map/map_point.h>

namespace orb_slam3 {
namespace frame {

CovisibilityGraphNode::CovisibilityGraphNode(const frame::KeyFrame * frame) : frame_(frame) {

}

void CovisibilityGraphNode::Update() {

  std::unordered_set<map::MapPoint *> local_map_points_;
  std::unordered_map<KeyFrame *, size_t> connected_frame_weights;
  frame_->ListMapPoints(local_map_points_);
  for (auto map_point:local_map_points_) {
    for (const auto& obs: map_point->Observations()) {
      if (obs.first == this->frame_)
        continue;
      ++connected_frame_weights[obs.first];
    }
  }
  std::vector<std::pair<size_t, KeyFrame *>> weight_frame_pairs;

  weight_frame_pairs.reserve(connected_frame_weights.size());
  for (auto & fp: connected_frame_weights)
    weight_frame_pairs.emplace_back(fp.second, fp.first);

  std::sort(weight_frame_pairs.begin(), weight_frame_pairs.end());

  std::unique_lock<std::mutex> lock_guard(mutex_);
  sorted_connected_frames_.clear();
  sorted_weights_.clear();
  sorted_weights_.reserve(weight_frame_pairs.size());
  sorted_connected_frames_.reserve(weight_frame_pairs.size());
  for (auto p = weight_frame_pairs.rbegin(); p != weight_frame_pairs.rend() && p->first >= 15; ++p) {
    sorted_weights_.push_back(p->first);
    sorted_connected_frames_.push_back(p->second);
  }
}

std::unordered_set<KeyFrame *> CovisibilityGraphNode::GetCovisibleKeyFrames( size_t count) const {
  std::lock_guard<std::mutex> m(mutex_);
  std::unordered_set<KeyFrame *> result;
  for (size_t i = 0; i< std::min(sorted_connected_frames_.size(), count); ++i)
    if (!sorted_connected_frames_[i]->IsBad())
      result.insert(sorted_connected_frames_[i]);
  return result;

}

}
}