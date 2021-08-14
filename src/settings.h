//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_SETTINGS_H_
#define ORB_SLAM3_SRC_SETTINGS_H_

#include <messages/message_type.h>
#include <cstddef>

namespace orb_slam3 {

class Settings {
 public:
  static Settings & Get() {
    static Settings s;
    return s;
  }
  bool MessageRequested(messages::MessageType type) const;
  void RequestMessage(messages::MessageType type);
  void UnrequestMessage(messages::MessageType type);
 private:
  Settings() = default;
 private:
  size_t requested_messages_;

};

}
#endif //ORB_SLAM3_SRC_SETTINGS_H_
