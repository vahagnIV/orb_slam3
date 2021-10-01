//
// Created by vahagn on 01/10/2021.
//

#ifndef ORB_SLAM3_SRC_FACTORIES_FRAME_FACTORY_H_
#define ORB_SLAM3_SRC_FACTORIES_FRAME_FACTORY_H_

#include <frame/frame.h>

namespace orb_slam3 {
namespace factories {

class FrameFactory {
 public:
  static frame::Frame * Create(frame::FrameType & type);
};

}
}
#endif //ORB_SLAM3_SRC_FACTORIES_FRAME_FACTORY_H_
