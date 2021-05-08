//
// Created by vahagn on 16/03/2021.
//

#include "local_mapper.h"
#include <frame/visible_map_point.h>

namespace orb_slam3 {

LocalMapper::LocalMapper(map::Atlas * atlas) : PositionObserver(), atlas_(atlas), thread_(nullptr) {}

LocalMapper::~LocalMapper() {
  Stop();
}

void LocalMapper::Run() {
  while (!cancelled_) {
    UpdateMessage message;
    GetUpdateQueue().wait_dequeue(message);
    switch (message.type) {
      case PositionMessageType::Final:continue;
      case PositionMessageType::Initial:continue;
      case PositionMessageType::Update: {
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

void LocalMapper::MapPointCulling(unordered_set<map::MapPoint *> & map_points) {

}

void LocalMapper::EraseMapPoint(map::MapPoint * map_point) {
  for(auto obs: map_point->Observations())
    break;
//    obs.first->Er
}

bool LocalMapper::CreateNewMapPoints(frame::FrameBase * frame) {
  //TODO: decide the number of covisible frames from the type, i.e. Monocular 20,  else 10
  auto covisible_frames = frame->CovisibilityGraph().GetCovisibleKeyFrames(20);
  frame->CreateNewMapPoints(nullptr, std::unordered_set<map::MapPoint *>());
//  const precision_t ratioFactor = 1.5f * frame->GetFeatureExtractor()->GetScaleFactor();

  std::unordered_set<map::MapPoint *> new_map_points;
  frame->CreateNewMapPoints(frame, new_map_points);
  return true;
//  if (frame->CreateNewMapPoints())

}

}