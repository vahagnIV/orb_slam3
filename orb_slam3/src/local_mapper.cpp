//
// Created by vahagn on 16/03/2021.
//

#include "local_mapper.h"

namespace orb_slam3 {

void LocalMapper::Run() {
  while (true) {
    UpdateMessage message;
    GetUpdateQueue().wait_dequeue(message);
    switch (message.type) {
      case MessageType::Final:return;
      case MessageType::Initial:continue;
      case MessageType::Update: {

      }

    }
  }
}

}