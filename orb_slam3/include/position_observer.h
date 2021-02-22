//
// Created by vahagn on 22/02/21.
//

#ifndef ORB_SLAM3_POISITION_OBSERVER_H
#define ORB_SLAM3_POISITION_OBSERVER_H
#include <concurrentqueue/blockingconcurrentqueue.h>
#include <frame/frame_base.h>

namespace orb_slam3 {

enum MessageType{
  Initial,
  Update,
  Final
};

struct UpdateMessage{
  MessageType type;
  std::shared_ptr<const frame::FrameBase> frame;
};

typedef moodycamel::BlockingConcurrentQueue<UpdateMessage> UpdateQueue;

class PositionObserver {
 public:
  PositionObserver(): queue_(){}
  UpdateQueue * GetUpdateQueue(){
    return &queue_;
  }

 private:
  UpdateQueue queue_;

};

}

#endif //ORB_SLAM3_POISITION_OBSERVER_H
