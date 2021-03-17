//
// Created by vahagn on 16.03.21.
//
// === stl ===
#include <algorithm>

// === orb-slam3 ===
#include <frame/covisibility_container.h>
#include <frame/frame_base.h>

namespace orb_slam3 {
namespace frame {

void CovisibilityContainer::AddConnection(FrameBase *frame) {
  ++connected_frame_weights[frame];
}

void CovisibilityContainer::RemoveConnection(FrameBase *frame) {
  decltype(connected_frame_weights)::iterator it = connected_frame_weights.find(frame);
  if (it == connected_frame_weights.end())
    return;
  --it->second;
  if (it->second == 0)
    connected_frame_weights.erase(it);
}

/*CovisibilityContainer::CovisibilityContainer(FrameBase *frame)
    : connected_frame_weights(),
      sorted_connected_frames(),
      sorted_weights(),
      frame_(frame),
      mutex_() {
}

void CovisibilityContainer::ComputeWeights(std::unordered_map<FrameBase *, unsigned int> &out_weights) {
  for (const auto &mp_id: frame_->MapPoints()) {
    if (nullptr == mp_id.second)
      continue;
    for (const auto &fr_id: mp_id.second->Observations()) {
      if (fr_id.first == frame_)
        continue;
      ++out_weights.at(fr_id.first);
    }
  }

  if (out_weights.empty()) {

  }
}
*/
void CovisibilityContainer::Update() {
  sorted_connected_frames.clear();
  sorted_weights.clear();

  std::vector<std::pair<unsigned, FrameBase *>> weight_frame_pairs;
  weight_frame_pairs.reserve(connected_frame_weights.size());
  for (auto &fp: connected_frame_weights)
    weight_frame_pairs.emplace_back(fp.second, fp.first);

  std::sort(weight_frame_pairs.begin(), weight_frame_pairs.end());

  for (auto &p: weight_frame_pairs) {
    sorted_weights.push_back(p.first);
    sorted_connected_frames.push_back(p.second);
  }
}

}
}