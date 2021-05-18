//
// Created by vahagn on 12/8/20.
//

#include <cassert>

#include "tracker.h"
#include "constants.h"
#include "logging.h"
#include <frame/key_frame.h>
#include <optimization/bundle_adjustment.h>

#include "local_mapper.h"
#include "debug/debug_utils.h"

namespace orb_slam3 {

Tracker::Tracker(orb_slam3::map::Atlas * atlas)
    : atlas_(atlas),
      velocity_is_valid_(false),
      last_frame_(nullptr),
      state_(NOT_INITIALIZED) {
  kf_counter = 1000;
}

Tracker::~Tracker() {
  delete atlas_;
}

frame::KeyFrame * Tracker::ListLocalKeyFrames(frame::Frame * current_frame,
                                              unordered_set<frame::KeyFrame *> & out_local_keyframes) {
  out_local_keyframes.clear();
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
    out_local_keyframes.insert(kf_count.first);
    if (max_count < kf_count.second) {
      max_count = kf_count.second;
      max_covisible_key_frame = kf_count.first;
    }
  }

  for (auto kf: key_frame_counter) {
    auto covisible_frames = kf.first->GetCovisibilityGraph().GetCovisibleKeyFrames(10);
    if (out_local_keyframes.size() >= 80)
      break;
    std::copy(covisible_frames.begin(),
              covisible_frames.end(),
              std::inserter(out_local_keyframes, out_local_keyframes.begin()));
  }

  return max_covisible_key_frame;
}

bool Tracker::TrackWithMotionModel(frame::Frame * frame) {
  if (!velocity_is_valid_)
    return false;
  std::unordered_set<map::MapPoint *> all_candidate_map_points;
  PredictAndSetNewFramePosition(frame);
  last_frame_->ListMapPoints(all_candidate_map_points);
  return frame->EstimatePositionByProjectingMapPoints(all_candidate_map_points);
}

bool Tracker::TrackWithReferenceKeyFrame(frame::Frame * frame) {
  frame->SetPosition(reference_keyframe_->GetPosition());
  return frame->FindMapPointsFromReferenceKeyFrame(reference_keyframe_);
}

TrackingResult Tracker::TrackInOkState(frame::Frame * frame) {

  assert(OK == state_);
  ++kf_counter;

  bool tracked = (TrackWithMotionModel(frame) || TrackWithReferenceKeyFrame(frame));

  if (!tracked) {
    // TODO: go to relocalization
    delete frame;
    return TrackingResult::TRACKING_FAILED;
  }

  frame->OptimizePose();
  if (frame->Id() - last_frame_->Id() == 1)
    ComputeVelocity(frame, last_frame_);

  debug::DrawCommonMapPoints(frame->GetFilename(),
                             reference_keyframe_->GetFilename(),
                             dynamic_cast<frame::monocular::MonocularFrame *>(frame),
                             dynamic_cast<frame::monocular::MonocularKeyFrame *>(reference_keyframe_));

  std::unordered_set<frame::KeyFrame *> local_keyframes;
  frame::KeyFrame * reference_keyframe = ListLocalKeyFrames(frame, local_keyframes);
  if (reference_keyframe)
    reference_keyframe_ = reference_keyframe;

  frame::Frame::MapPointSet current_frame_map_points, local_map_points_except_current;
  frame->ListMapPoints(current_frame_map_points);
  for (auto mp: current_frame_map_points)
    mp->IncreaseVisible();

  for (auto keyframe: local_keyframes) {
    frame::Frame::MapPointSet key_frame_map_points;
    keyframe->ListMapPoints(key_frame_map_points);
    for (auto mp: key_frame_map_points) {
      if (current_frame_map_points.find(mp) == current_frame_map_points.end())
        local_map_points_except_current.insert(mp);
    }
  }

  logging::RetrieveLogger()->debug("Local mp count {}", local_map_points_except_current.size());

  std::list<frame::VisibleMapPoint> visible_map_points;
  frame->FilterVisibleMapPoints(local_map_points_except_current,
                                visible_map_points,
                                frame->GetSensorConstants()->projection_search_radius_multiplier);
  for (auto & visible_map_point: visible_map_points)
    visible_map_point.map_point->IncreaseVisible();

  frame->SearchInVisiblePoints(visible_map_points);
  frame->OptimizePose();
  frame::Frame::MapPointSet map_points;
  frame->ListMapPoints(map_points);
  for (auto map_point: map_points)
    map_point->IncreaseFound();

  //TODO: Add keyframe if necessary
  if (NeedNewKeyFrame(frame)) {
    auto keyframe = frame->CreateKeyFrame();
    atlas_->GetCurrentMap()->AddKeyFrame(keyframe);
    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Update, .frame = keyframe});

    // TODO: remove the following line
    (dynamic_cast<LocalMapper *>(*(observers_.begin())))->RunIteration();
  }
//  ComputeVelocity(frame, last_frame_);
  ReplaceLastFrame(frame);
  return TrackingResult::OK;
}

void Tracker::ReplaceLastFrame(frame::Frame * frame) {
  auto tmp = last_frame_;
  last_frame_ = frame;
  delete tmp;
}

bool Tracker::NeedNewKeyFrame(frame::Frame * frame) {

  std::unordered_set<map::MapPoint *> m;
  frame->ListMapPoints(m);
  bool need = kf_counter >= 2;// && m.size() > 40;
  if (need)
    kf_counter = 0;

  return need;
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
    state_ = OK;
    last_frame_ = frame;

    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Initial, .frame=initial_key_frame});
    this->NotifyObservers(UpdateMessage{.type = PositionMessageType::Update, .frame=initial_key_frame});

    // TODO: remove the following line in multithreading
    (dynamic_cast<LocalMapper *>(*(observers_.begin())))->RunIteration();

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

}  // namespace orb_slam3
