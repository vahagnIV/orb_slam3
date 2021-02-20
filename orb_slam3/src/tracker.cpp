//
// Created by vahagn on 12/8/20.
//

#include "tracker.h"

#include <constants.h>

namespace orb_slam3 {

Tracker::Tracker()
    : atlas_(new map::Atlas),
      last_frame_(nullptr),
      initial_frame_(nullptr),
      state_(NOT_INITIALIZED) {}

Tracker::~Tracker() { delete atlas_; }

TrackingResult Tracker::Track(const std::shared_ptr<FrameBase> & frame) {
//  Map *current_map = atlas_->GetCurrentMap();
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
        map::Map * current_map = atlas_->GetCurrentMap();
        current_map->AddKeyFrame(frame);
        current_map->SetInitialKeyFrame(last_frame_);
        for (auto & mp: frame->MapPoints())
          if (mp) {
            mp->AddFrame(frame);
            mp->AddFrame(last_frame_);
          }
        last_frame_ = frame;
        state_ = OK;
      }
    }
      break;

    default:break;
  }
  return TrackingResult::OK;
}

bool Tracker::TrackReferenceKeyFrame() { return false; }

}  // namespace orb_slam3