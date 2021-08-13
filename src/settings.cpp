//
// Created by vahagn on 13/08/2021.
//

#include "settings.h"

namespace orb_slam3{

bool Settings::MessageRequested(messages::MessageType type) const {
  return requested_messages_ & type;
}

void Settings::RequestMessage(messages::MessageType type) {
  requested_messages_ |= type;
}

void Settings::UnrequestMessage(messages::MessageType type) {
  requested_messages_ &= ~type;
}

}