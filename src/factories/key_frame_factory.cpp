//
// Created by vahagn on 29/09/2021.
//

#include "key_frame_factory.h"
#include <frame/monocular/monocular_key_frame.h>

namespace orb_slam3 {
namespace factories {

frame::KeyFrame * KeyFrameFactory::Create(frame::FrameType & type) {
  switch (type) {
    case frame::FrameType::MONOCULAR:
      return new frame::monocular::MonocularKeyFrame();
    default:
      return nullptr;
  }
  return nullptr;
}

}
}