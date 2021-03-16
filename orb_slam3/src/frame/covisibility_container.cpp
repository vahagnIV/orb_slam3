//
// Created by vahagn on 16.03.21.
//
// === stl ===
#include <algorithm>

// === orb-slam3 ===
#include <frame/covisibility_container.h>

namespace orb_slam3 {
namespace frame {

void CovisibilityContainer::Update(std::unordered_map<FrameBase *, unsigned> &frame_weights) {
  connected_frame_weights.clear();

  std::copy_if(frame_weights.begin(),
               frame_weights.end(),
               std::inserter(connected_frame_weights,
                             connected_frame_weights.begin()),
               [](const std::pair<FrameBase *, size_t> &pair) -> bool { return pair.second > 15; });
  connected_frames.resize(connected_frame_weights.size());
  std::transform(connected_frame_weights.begin(),
                 connected_frame_weights.end(),
                 connected_frames.begin(),
                 [](const std::pair<FrameBase *, size_t> &pair) { return pair.first; });
}

}
}