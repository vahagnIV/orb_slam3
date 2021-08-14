//
// Created by vahagn on 13/08/2021.
//

#include "message_processor.h"

namespace orb_slam3{
namespace messages{

void MessageProcessor::Enqueue(BaseMessage * message) {
  message_queue_.enqueue(message);
}

bool MessageProcessor::Dequeue(BaseMessage *& message) {
  return message_queue_.try_dequeue(message);
}

}
}