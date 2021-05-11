//
// Created by vahagn on 19/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OBSERVER_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OBSERVER_H_

#include <concurrentqueue/blockingconcurrentqueue.h>

namespace orb_slam3 {

template<typename MessageType>
class Observer {
 public:
  typedef moodycamel::BlockingConcurrentQueue<MessageType> UpdateQueue;
  Observer(): queue_(){}
  UpdateQueue & GetUpdateQueue(){
    return queue_;
  }

 private:
  UpdateQueue queue_;

};

}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OBSERVER_H_
