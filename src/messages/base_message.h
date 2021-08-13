//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_BASE_MESSAGE_H_
#define ORB_SLAM3_SRC_MESSAGES_BASE_MESSAGE_H_

#include <atomic>

#include "message_type.h"

namespace orb_slam3 {
namespace messages {

class BaseMessage {
 public:
  BaseMessage();
  virtual ~BaseMessage() = default;
 public:
  std::size_t Nonce() const;
  virtual MessageType Type() const = 0;
 private:
  std::size_t nonce_;
 private:
  static std::atomic_uint64_t next_nonce_;

};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_BASE_MESSAGE_H_
