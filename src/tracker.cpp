//
// Created by vahagn on 12/8/20.
//

#include <cassert>

#include "tracker.h"
#include "constants.h"
#include "logging.h"
#include <frame/key_frame.h>
#include <optimization/bundle_adjustment.h>

namespace orb_slam3 {

Tracker::Tracker(orb_slam3::map::Atlas * atlas)
    : local_mapper_(atlas),
      atlas_(atlas),
      velocity_is_valid_(false),
      last_frame_(nullptr),
      state_(NOT_INITIALIZED) {
  kf_counter = 1000;
}

Tracker::~Tracker() {
  delete atlas_;
}

void Tracker::UpdateLocalPoints() {
  local_map_points_.clear();
  for (auto key_frame: local_key_frames_) {
    key_frame->ListMapPoints(local_map_points_);
  }
  logging::RetrieveLogger()->debug("Local mp count {}", local_map_points_.size());
}

bool Tracker::TrackWithMotionModel(frame::Frame * frame) {
  std::unordered_set<map::MapPoint *> all_candidate_map_points;
  PredictAndSetNewFramePosition(frame);
  last_frame_->ListMapPoints(all_candidate_map_points);
  return true;
// TODO: return frame->FindNewMapPointsAndAdjustPosition(all_candidate_map_points);
}

TrackingResult Tracker::TrackInOkState(frame::Frame * frame) {

  assert(OK == state_);
  ++kf_counter;

  bool tracked = false;
  if (!velocity_is_valid_) {
    frame->SetPosition(last_frame_->GetPosition());
// TODO:    tracked = frame->TrackWithReferenceKeyFrame(reference_keyframe_);
  } else {
    tracked = TrackWithMotionModel(frame);
    if (!tracked) {
      frame->SetPosition(last_frame_->GetPosition());
//TODO:      tracked = frame->TrackWithReferenceKeyFrame(reference_keyframe_);
    }
  }

  if (!tracked) {
    delete frame;
    return TrackingResult::TRACKING_FAILED;
  }
  if (frame->Id() - last_frame_->Id() == 1)
    ComputeVelocity(frame, last_frame_);

  UpdateLocalMap(frame);
//TODO:  frame->SearchLocalPoints(local_map_points_);
  //TODO: Add keyframe if necessary
  if (NeedNewKeyFrame(frame)) {
    auto keyframe = frame->CreateKeyFrame();

    if (local_mapper_.CreateNewMapPoints(keyframe)) {

      UpdateCovisibilityConnections();
      last_key_frame_ = keyframe;
      atlas_->GetCurrentMap()->AddKeyFrame(keyframe);
      reference_keyframe_ = keyframe;
    }
//    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Update, .frame = frame});
  }
//  ComputeVelocity(frame, last_frame_);
  last_frame_ = frame;
  return TrackingResult::OK;
}

bool Tracker::NeedNewKeyFrame(frame::Frame * frame) {

  std::unordered_set<map::MapPoint *> m;
  frame->ListMapPoints(m);
  bool need = local_map_points_.size() < 300 || kf_counter >= 2;// && m.size() > 40;
  if (need)
    kf_counter = 0;

  return need;
}

void Tracker::UpdateCovisibilityConnections() {
  for (auto frame: local_key_frames_)
    frame->GetCovisibilityGraph().Update();
}

void Tracker::OptimizePose(FrameBase * frmae) {

}

TrackingResult Tracker::TrackInFirstImageState(frame::Frame * frame) {

  assert(FIRST_IMAGE == state_);
  assert(frame->IsValid());

  if (frame->Link(last_frame_)) {
    map::Map * current_map = atlas_->GetCurrentMap();

    frame::KeyFrame * initial_key_frame = last_frame_->CreateKeyFrame();
    initial_key_frame->SetInitial(true);

    frame::KeyFrame * current_key_frame = frame->CreateKeyFrame();

    current_map->SetInitialKeyFrame(initial_key_frame);
    current_map->AddKeyFrame(current_key_frame);

    std::unordered_set<frame::KeyFrame *> key_frames{initial_key_frame, current_key_frame};
    std::unordered_set<map::MapPoint *> map_points;
    current_key_frame->ListMapPoints(map_points);

    optimization::BundleAdjustment(key_frames, map_points, 30);

    reference_keyframe_ = current_key_frame;
    local_mapper_.AddKeyFrame(initial_key_frame);
    local_mapper_.AddKeyFrame(current_key_frame);
    state_ = OK;
    last_frame_ = frame;
    /*this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Initial, .frame=frame});*/
  } else {
    delete frame;
    return TrackingResult::TRACKING_FAILED;
  }

  return TrackingResult::OK;
}

TrackingResult Tracker::TrackInNotInitializedState(frame::Frame * frame) {

  assert(NOT_INITIALIZED == state_);
  assert(frame->IsValid());
  frame->SetIdentity();
//  frame->SetInitial(true);
  state_ = FIRST_IMAGE;
  last_frame_ = frame;
  return TrackingResult::OK;
}

TrackingResult Tracker::Track(frame::Frame * frame) {

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

void Tracker::ComputeVelocity(const geometry::RigidObject * object2, const geometry::RigidObject * object1) {
  velocity_ = object1->GetInversePosition().T - object2->GetInversePosition().T;
  angular_velocity_ = object1->GetPosition().R * object2->GetInversePosition().R;
  velocity_is_valid_ = true;
}

void Tracker::PredictAndSetNewFramePosition(frame::Frame * frame) const {
  geometry::Pose pose;
  pose.R = angular_velocity_ * last_frame_->GetPosition().R;
  pose.T = -pose.R * (last_frame_->GetInversePosition().T + velocity_);
  frame->SetPosition(pose);
}

void Tracker::UpdateLocalMap(frame::Frame * current_frame) {
  UpdateLocalKeyFrames(current_frame);
  UpdateLocalPoints();
}

void Tracker::UpdateLocalKeyFrames(frame::Frame * current_frame) {
  local_key_frames_.clear();
  std::unordered_set<map::MapPoint *> frame_map_points;
  current_frame->ListMapPoints(frame_map_points);
  std::unordered_map<frame::KeyFrame *, unsigned> key_frame_counter;
  for (auto & map_point: frame_map_points) {
    for (auto observation: map_point->Observations()) {
      ++key_frame_counter[observation.first];
    }
  }

  frame::KeyFrame * max_covisible_key_frame = nullptr;
  unsigned max_count = 0;
  for (auto & kf_count: key_frame_counter) {
    local_key_frames_.insert(kf_count.first);
    if (max_count < kf_count.second) {
      max_count = kf_count.second;
      max_covisible_key_frame = kf_count.first;
    }
  }

  for (auto kf: key_frame_counter) {
    auto covisible_frames = kf.first->GetCovisibilityGraph().GetCovisibleKeyFrames(10);
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
