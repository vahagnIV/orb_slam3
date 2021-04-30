//
// Created by vahagn on 12/8/20.
//

#include <cassert>

#include "tracker.h"
#include <constants.h>
#include <optimization/bundle_adjustment.h>
#include <logging.h>

namespace orb_slam3 {

Tracker::Tracker(orb_slam3::map::Atlas * atlas)
    : local_mapper_(atlas),
      atlas_(atlas),
      last_frame_(nullptr),
      initial_frame_(nullptr),
      state_(NOT_INITIALIZED) {
}

Tracker::~Tracker() {
  delete atlas_;
}

void Tracker::UpdateLocalPoints() {
  local_map_points_.clear();
  for (auto frame: local_key_frames_) {
    frame->ListMapPoints(local_map_points_);
  }
}

TrackingResult Tracker::TrackInOkState(FrameBase * frame) {

  assert(OK == state_);
  PredictAndSetNewFramePosition(frame);

  if (!frame->TrackWithMotionModel(last_frame_)) {
    frame->SetPosition(*last_frame_->GetPose());
    if (!frame->TrackWithReferenceKeyFrame(reference_keyframe_)) {
      logging::RetrieveLogger()->info("tracking failed");
      delete frame;
//      exit(1);
      return TrackingResult::TRACK_LM_FAILED;
    }
  }
//  ComputeVelocity(frame, last_frame_);
  UpdateLocalMap(frame);
  frame->SearchLocalPoints(local_map_points_);
  //TODO: Add keyframe if necessary
  if (NeedNewKeyFrame(frame)) {

    if (local_mapper_.CreateNewMapPoints(frame)) {

      UpdateCovisibilityConnections();
      last_key_frame_ = frame;
      atlas_->GetCurrentMap()->AddKeyFrame(frame);
    }
//    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Update, .frame = frame});
  }
  ComputeVelocity(frame, last_frame_);
  last_frame_ = frame;
  return TrackingResult::OK;
}

bool Tracker::NeedNewKeyFrame(frame::FrameBase * frame) {
  std::unordered_set<map::MapPoint *> m;
  frame->ListMapPoints(m);
  bool need = kf_counter >= 4;// && m.size() > 40;
  if (need)
    kf_counter = 0;

  return need;
}

void Tracker::UpdateCovisibilityConnections() {
  for(auto frame: local_key_frames_)
    frame->CovisibilityGraph().Update();
}

TrackingResult Tracker::TrackInFirstImageState(FrameBase * frame) {

  assert(FIRST_IMAGE == state_);
  assert(frame->IsValid());

  if (frame->Link(last_frame_)) {
    map::Map * current_map = atlas_->GetCurrentMap();
    current_map->AddKeyFrame(frame);
    current_map->AddKeyFrame(last_frame_);
    current_map->SetInitialKeyFrame(last_frame_);
    local_key_frames_.insert(frame);
    local_key_frames_.insert(last_frame_);
    UpdateCovisibilityConnections();
    frame->ListMapPoints(local_map_points_);
    reference_keyframe_ = frame;

    state_ = OK;
    ComputeVelocity(frame, last_frame_);


    last_frame_ = last_key_frame_ = frame;
    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Initial, .frame=frame});
  } else {
    delete frame;
    return TrackingResult::TRACKING_FAILED;
  }

  return TrackingResult::OK;
}

TrackingResult Tracker::TrackInNotInitializedState(FrameBase * frame) {

  assert(NOT_INITIALIZED == state_);
  assert(frame->IsValid());
  initial_frame_ = frame;
  frame->InitializeIdentity();
  frame->SetInitial(true);
  state_ = FIRST_IMAGE;
  last_frame_ = frame;
  return TrackingResult::OK;
}

TrackingResult Tracker::Track(FrameBase * frame) {
  ++kf_counter;
  frame::FrameBase * new_key_frame;
  if (GetUpdateQueue().try_dequeue(new_key_frame)) {
    last_key_frame_ = new_key_frame;
    atlas_->GetCurrentMap()->AddKeyFrame(last_key_frame_);
  }

  if (!frame->IsValid()) {
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

void Tracker::ComputeVelocity(const FrameBase * frame2, const FrameBase * frame1) {
  velocity_ = frame1->GetInversePose()->T - frame2->GetInversePose()->T;
  angular_velocity_ = frame1->GetPose()->R * frame2->GetInversePose()->R;
}

void Tracker::PredictAndSetNewFramePosition(FrameBase * frame) const {
  geometry::Pose pose;
  pose.R = angular_velocity_ * last_frame_->GetPose()->R;
  pose.T = -pose.R * (last_frame_->GetInversePose()->T + velocity_);
  frame->SetPosition(pose);
}

void Tracker::UpdateLocalMap(frame::FrameBase * current_frame) {
  UpdateLocalKeyFrames(current_frame);
  UpdateLocalPoints();
}

void Tracker::UpdateLocalKeyFrames(frame::FrameBase * current_frame) {
  local_key_frames_.clear();
  std::unordered_set<map::MapPoint *> frame_map_points;
  current_frame->ListMapPoints(frame_map_points);
  std::unordered_map<frame::FrameBase *, unsigned> key_frame_counter;
  for (auto & map_point: frame_map_points) {
    for (auto observation: map_point->Observations()) {
      ++key_frame_counter[observation.first];
    }
  }

  FrameBase * max_covisible_key_frame = nullptr;
  unsigned max_count = 0;
  for (auto & kf_count: key_frame_counter) {
    local_key_frames_.insert(kf_count.first);
    if (max_count < kf_count.second) {
      max_count = kf_count.second;
      max_covisible_key_frame = kf_count.first;
    }
  }

  for (auto kf: key_frame_counter) {
    auto covisible_frames = kf.first->CovisibilityGraph().GetCovisibleKeyFrames(10);
    if (local_key_frames_.size() >= 80)
      break;
    std::copy(covisible_frames.begin(),
              covisible_frames.end(),
              std::inserter(local_key_frames_, local_key_frames_.begin()));
  }

  if (max_covisible_key_frame)
    reference_keyframe_ = max_covisible_key_frame;

}

}  // namespace orb_slam3
