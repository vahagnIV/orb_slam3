//
// Created by vahagn on 12/8/20.
//

// == orb-slam3 ===
#include "map.h"

namespace orb_slam3 {
namespace map {

void Map::AddKeyFrame(frame::KeyFrame * frame) {
  key_frames_.insert(frame);
}

void Map::SetInitialKeyFrame(frame::KeyFrame * frame) {
  initial_keyframe_ = frame;
}

}
}