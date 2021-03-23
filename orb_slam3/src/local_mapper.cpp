//
// Created by vahagn on 16/03/2021.
//

#include "local_mapper.h"

namespace orb_slam3 {

LocalMapper::LocalMapper(map::Atlas *atlas) : PositionObserver(), atlas_(atlas), thread_(nullptr) {}

LocalMapper::~LocalMapper() {
  Stop();
}

void LocalMapper::Run() {
  while (!cancelled_) {
    UpdateMessage message;
    GetUpdateQueue().wait_dequeue(message);
    switch (message.type) {
      case MessageType::Final:continue;
      case MessageType::Initial:continue;
      case MessageType::Update: {
        std::cout << message.frame->Id() << std::endl;
        CreateNewMapPoints(message.frame);
      }
    }
  }
}

void LocalMapper::Start() {
  if (thread_ != nullptr)
    return;
  cancelled_ = false;
  thread_ = new std::thread(&LocalMapper::Run, this);
}

void LocalMapper::Stop() {
  if (nullptr == thread_)
    return;
  cancelled_ = true;
  thread_->join();
  delete thread_;
  thread_ = nullptr;
}

void LocalMapper::CreateNewMapPoints(const std::shared_ptr<frame::FrameBase> &frame) const {
  frame->FindNewMapPoints();
}

}