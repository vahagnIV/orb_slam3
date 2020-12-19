//
// Created by vahagn on 12/8/20.
//

#include "map.h"

namespace orb_slam3 {

//void Map::AddKeyFrame(const FrameBase *frame) {
//  key_frames_.push_back(frame);
//}

void Map::SetLastFrame(FrameBase *frame) noexcept {
  if (last_frame_ && !key_frames_.empty() && last_frame_ != key_frames_.back())
    delete frame;

  last_frame_ = frame;
}

void Map::AcceptLastFrame() noexcept {
  if (last_frame_ && (key_frames_.empty() || last_frame_ != key_frames_.back()))
    key_frames_.push_back(last_frame_);
}

Map::~Map() {
  for (FrameBase *frame: key_frames_)
    delete frame;

}

}