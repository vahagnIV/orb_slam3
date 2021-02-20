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
        state_ = FIRST_IMAGE;
      }
    }
      break;
    case FIRST_IMAGE: {
      frame->SetPrevious(last_frame_);
      if (frame->InitializePositionFromPrevious()) {
        last_frame_->InitializeIdentity();
        map::Map * current_map = atlas_->GetCurrentMap();
        current_map->AddKeyFrame(frame);
        current_map->AddKeyFrame(last_frame_);
        state_ = OK;
      }
    }
      break;

    default:break;
  }

  /* IMU stuff skipping for now ... */

  /*if (current_map->GetLastFrame() == nullptr) {

    if (frame->FeatureCount() < MINIMAL_FEATURE_COUNT_PER_FRAME)
      return TrackingResult::Ignore;
    frame->InitializeIdentity();
    current_map->SetLastFrame(frame);
    current_map->AcceptLastFrame();

  } else {
    frame->SetReferenceFrame(current_map->GetLastFrame());
  }*/
  return TrackingResult::OK;
}

bool Tracker::TrackReferenceKeyFrame() { return false; }

}  // namespace orb_slam3