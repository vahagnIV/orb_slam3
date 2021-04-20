//
// Created by vahagn on 12/8/20.
//

#include <cassert>

#include "tracker.h"
#include <constants.h>
#include <optimization/bundle_adjustment.h>

namespace orb_slam3 {

Tracker::Tracker(orb_slam3::map::Atlas * atlas)
    : atlas_(atlas),
      last_frame_(nullptr),
      initial_frame_(nullptr),
      state_(NOT_INITIALIZED)
{
}

Tracker::~Tracker() {
  delete atlas_;
}

TrackingResult Tracker::TrackInOkState(const std::shared_ptr<FrameBase> & frame) {

  assert(OK == state_);
  assert(frame->IsValid());
  if (! frame->TrackWithReferenceKeyFrame(last_frame_)) {
    state_ = RECENTLY_LOST;
    return TrackingResult::TRACKING_FAILED;
  }
  if (! frame->TrackLocalMap()) {
    return TrackingResult::TRACK_LM_FAILED;
  }
  map::Map * current_map = atlas_->GetCurrentMap();
  current_map->AddKeyFrame(frame);
  last_frame_ = frame;
  NotifyObservers(last_frame_, MessageType::Update);
  return TrackingResult::OK;
}

TrackingResult Tracker::TrackInFirstImageState(const std::shared_ptr<FrameBase> & frame) {

  assert(FIRST_IMAGE == state_);
  assert(frame->IsValid());
  if (frame->Link(last_frame_)) {
    map::Map * current_map = atlas_->GetCurrentMap();
    current_map->AddKeyFrame(frame);
    current_map->SetInitialKeyFrame(last_frame_);

    state_ = OK;
    last_frame_ = last_key_frame_ = frame;
    NotifyObservers(frame, MessageType::Initial);
  }
  return TrackingResult::OK;
}

TrackingResult Tracker::TrackInNotInitializedState(const std::shared_ptr<FrameBase> & frame) {

  assert(NOT_INITIALIZED == state_);
  assert(frame->IsValid());
  initial_frame_ = frame;
  frame->InitializeIdentity();
  frame->SetInitial(true);
  state_ = FIRST_IMAGE;
  last_frame_ = frame;
  return TrackingResult::OK;
}

TrackingResult Tracker::Track(const std::shared_ptr<FrameBase> & frame) {

  if (! frame->IsValid()) {
    return TrackingResult::INVALID_FRAME;
  }
  TrackingResult res = TrackingResult::OK;
  switch (state_) {
    case NOT_INITIALIZED: {
      res = TrackInNotInitializedState(frame);
      break;
    } case FIRST_IMAGE: {
      res = TrackInFirstImageState(frame);
      break;
    } case OK: {
      res = TrackInOkState(frame);
      break;
    } default: {
    }
  }
  return res;
}

void Tracker::NotifyObservers(const std::shared_ptr<FrameBase> & frame, MessageType type) {
  for (PositionObserver * observer: observers_) {
    observer->GetUpdateQueue().enqueue(UpdateMessage{type:type, frame:frame});
  }
}

}  // namespace orb_slam3
