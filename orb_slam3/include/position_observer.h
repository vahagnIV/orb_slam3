//
// Created by vahagn on 22/02/21.
//

#ifndef ORB_SLAM3_POISITION_OBSERVER_H
#define ORB_SLAM3_POISITION_OBSERVER_H
#include <observer.h>
#include <frame/frame_base.h>

namespace orb_slam3 {

enum PositionMessageType {
  Initial,
  Update,
  Final
};

struct UpdateMessage {
  PositionMessageType type;
  frame::FrameBase * frame;
};

class PositionObserver : public Observer<UpdateMessage> {

};

}

#endif //ORB_SLAM3_POISITION_OBSERVER_H
