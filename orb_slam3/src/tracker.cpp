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
      state_(NOT_INITIALIZED) {
}

Tracker::~Tracker() {
  delete atlas_;
}

TrackingResult Tracker::TrackInOkState(const std::shared_ptr<FrameBase> & frame) {

  assert(OK == state_);
  geometry::Pose pose;
  pose.R = angular_velocity_ * last_frame_->GetPose()->R;
  pose.T = -pose.R * (last_frame_->GetInversePose()->T + velocity_);
  frame->SetPosition(pose);

  if (!frame->TrackLocalMap(last_key_frame_)) {
    frame->SetPosition(*last_frame_->GetPose());
    if (!(frame->TrackWithReferenceKeyFrame(last_key_frame_) && frame->TrackLocalMap(last_key_frame_))) {
      return TrackingResult::TRACK_LM_FAILED;
    }
  }

  velocity_ = frame->GetInversePose()->T - last_frame_->GetInversePose()->T;
  angular_velocity_ = frame->GetPose()->R * last_frame_->GetInversePose()->R;
  //TODO: Add keyframe if necessary
  if (NeedNewKeyFrame()) {
//    last_key_frame_ = frame;
    atlas_->GetCurrentMap()->AddKeyFrame(last_key_frame_);
    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Update, .frame = frame});
  }
  last_frame_ = frame;
  return TrackingResult::OK;
}

bool Tracker::NeedNewKeyFrame() {
  bool need = kf_counter >= 4;
  if(need)
    kf_counter = 0;
  return need;
}

TrackingResult Tracker::TrackInFirstImageState(const std::shared_ptr<FrameBase> & frame) {

  assert(FIRST_IMAGE == state_);
  assert(frame->IsValid());
  if (frame->Link(last_frame_)) {
    map::Map * current_map = atlas_->GetCurrentMap();
    current_map->AddKeyFrame(frame);
    current_map->AddKeyFrame(last_frame_);
    current_map->SetInitialKeyFrame(last_frame_);

    state_ = OK;
    velocity_ = frame->GetInversePose()->T - last_frame_->GetInversePose()->T;
    angular_velocity_ = frame->GetPose()->R * last_frame_->GetInversePose()->R;
    last_frame_ = last_key_frame_ = frame;
    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Initial, .frame=frame});
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
  ++kf_counter;
  std::shared_ptr<frame::FrameBase> new_key_frame;
  if (GetUpdateQueue().try_dequeue(new_key_frame)) {
    last_key_frame_ = new_key_frame;
    atlas_->GetCurrentMap()->AddKeyFrame(last_key_frame_);
  }

  if (! frame->IsValid()) {
    return TrackingResult::INVALID_FRAME;
  }
  TrackingResult res = TrackingResult::OK;
  switch (state_) {
    case NOT_INITIALIZED: {
      res = TrackInNotInitializedState(frame);
      break;
    }
    case FIRST_IMAGE: {
      res = TrackInFirstImageState(frame);
      break;
    }
    case OK: {
      res = TrackInOkState(frame);
      break;
    }
    default: {
    }
  }
  return res;
}

}  // namespace orb_slam3
