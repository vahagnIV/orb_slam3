//
// Created by vahagn on 12/8/20.
//

#include "tracker.h"
#include <constants.h>
#include <optimization/bundle_adjustment.h>
namespace orb_slam3 {

Tracker::Tracker()
    : atlas_(new map::Atlas),
      last_frame_(nullptr),
      initial_frame_(nullptr),
      state_(NOT_INITIALIZED) {}

Tracker::~Tracker() { delete atlas_; }

TrackingResult Tracker::Track(const std::shared_ptr<FrameBase> &frame) {
  switch (state_) {
    case NOT_INITIALIZED: {
      if (frame->IsValid()) {
        initial_frame_ = last_frame_ = frame;
        last_frame_->InitializeIdentity();
        state_ = FIRST_IMAGE;
      }
    }
      break;
    case FIRST_IMAGE: {
      if (frame->Link(last_frame_)) {
        map::Map *current_map = atlas_->GetCurrentMap();
        current_map->AddKeyFrame(frame);
        current_map->SetInitialKeyFrame(last_frame_);

        last_frame_ = frame;
        state_ = OK;
        NotifyObservers(frame, MessageType::Initial);
      }
    }
      break;
    case OK: {
      if (frame->TrackWithReferenceKeyFrame(last_frame_)) {
        map::Map *current_map = atlas_->GetCurrentMap();
        current_map->AddKeyFrame(frame);
        last_frame_ = frame;
      }
    }
      break;

    default:break;

  }
  return TrackingResult::OK;
}

bool Tracker::TrackReferenceKeyFrame() { return false; }

void Tracker::NotifyObservers(const std::shared_ptr<const FrameBase> &frame, MessageType type) {
  for (PositionObserver *observer: observers_) {
    observer->GetUpdateQueue()->enqueue(UpdateMessage{type:type, frame:frame});
  }
}

}  // namespace orb_slam3