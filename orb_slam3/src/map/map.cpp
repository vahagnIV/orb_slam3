//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map/map.h"

namespace orb_slam3 {
namespace map {

void Map::AddKeyFrame(const std::shared_ptr<frame::FrameBase> & frame) {
  key_frames_.insert(frame);
}

}
}