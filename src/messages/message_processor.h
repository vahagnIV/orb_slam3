//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_MESSAGE_PROCESSOR_H_
#define ORB_SLAM3_SRC_MESSAGES_MESSAGE_PROCESSOR_H_

#include <concurrentqueue/concurrentqueue.h>
#include "base_message.h"
namespace orb_slam3 {
namespace messages {

class MessageProcessor {
  typedef moodycamel::ConcurrentQueue<BaseMessage *> MESSAGE_QUEUE;
 public:
  static MessageProcessor & Instance() {
    static MessageProcessor m;
    return m;
  }
  void Enqueue(BaseMessage * message);
  bool Dequeue(BaseMessage * & message);
 private:
  MessageProcessor() = default;
 private:
  MESSAGE_QUEUE message_queue_;

};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_MESSAGE_PROCESSOR_H_
