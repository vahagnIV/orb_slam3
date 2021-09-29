//
// Created by vahagn on 29/09/2021.
//

#ifndef ORB_SLAM3_SRC_FACTORIES_KEY_FRAME_FACTORY_H_
#define ORB_SLAM3_SRC_FACTORIES_KEY_FRAME_FACTORY_H_

#include <frame/key_frame.h>

namespace orb_slam3 {
namespace factories {

class KeyFrameFactory {
 public:
  static frame::KeyFrame * Create(frame::FrameType & type);
};

}
}
#endif //ORB_SLAM3_SRC_FACTORIES_KEY_FRAME_FACTORY_H_
