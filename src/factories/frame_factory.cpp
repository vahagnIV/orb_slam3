//
// Created by vahagn on 01/10/2021.
//

#include "frame_factory.h"
#include <frame/monocular/monocular_frame.h>


namespace orb_slam3 {
namespace factories {

frame::Frame * FrameFactory::Create(frame::FrameType & type) {
  switch (type) {
    case frame::FrameType::MONOCULAR:
      return new frame::monocular::MonocularFrame();
    default:
      return nullptr;

  }
  return nullptr;
}

}
}