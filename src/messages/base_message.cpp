//
// Created by vahagn on 13/08/2021.
//

#include "base_message.h"

namespace orb_slam3 {
namespace messages {

std::atomic_uint64_t BaseMessage::next_nonce_(0);

BaseMessage::BaseMessage() : nonce_(++next_nonce_) {

}

std::size_t BaseMessage::Nonce() const {
  return nonce_;
}

}
}