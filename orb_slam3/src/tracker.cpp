//
// Created by vahagn on 12/8/20.
//

#include "tracker.h"
#include <constants.h>
#include <optimization/bundle_adjustment.h>
namespace orb_slam3 {

Tracker::Tracker(orb_slam3::map::Atlas *atlas)
    : atlas_(atlas),
      last_frame_(nullptr),
      initial_frame_(nullptr),
      state_(NOT_INITIALIZED) {}

Tracker::~Tracker() { delete atlas_; }

TrackingResult Tracker::Track(const std::shared_ptr<FrameBase> & frame) {
  switch (state_) {
    case NOT_INITIALIZED: {
      if (frame->IsValid()) {
        initial_frame_ = frame;
        frame->InitializeIdentity();
        frame->SetInitial(true);
        state_ = FIRST_IMAGE;
        last_frame_ = frame;
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

      if (!frame->TrackWithReferenceKeyFrame(last_frame_)) {
        state_ = RECENTLY_LOST;
        return TRACKING_FAILED;
      }

      if (!frame->TrackLocalMap()) {
        return TRACK_LM_FAILED;
      }

      {

        map::Map *current_map = atlas_->GetCurrentMap();
        current_map->AddKeyFrame(frame);
        NotifyObservers(last_frame_, MessageType::Update);
      }
    }
      break;

    default:break;

  }

  return TrackingResult::OK;
}

void Tracker::NotifyObservers(const std::shared_ptr<FrameBase> & frame, MessageType type) {
  for (PositionObserver *observer: observers_) {
    observer->GetUpdateQueue().enqueue(UpdateMessage{type:type, frame:frame});
  }
}

}  // namespace orb_slam3