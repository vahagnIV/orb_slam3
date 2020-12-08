//
// Created by vahagn on 12/8/20.
//

#include "map.h"

namespace nvision {

void Map::AddKeyFrame(const std::shared_ptr<FrameBase> &frame) {
  key_frames_.push_back(frame);
}

}