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

bool Tracker::TrackLocalMap(const std::shared_ptr<FrameBase> & current_frame,
                            const std::unordered_set<FrameBase *> & local_key_frames,
                            const std::unordered_set<map::MapPoint *> & local_map_points) {

  /*std::unordered_set<map::MapPoint *> current_frame_map_points;

  std::transform(current_frame->MapPoints().begin(),
                 current_frame->MapPoints().end(),
                 std::inserter(current_frame_map_points, current_frame_map_points.begin()),
                 [](const std::pair<std::size_t, map::MapPoint *> & p) { return p.second; });

  for (auto & mp: local_map_points) {
    if (current_frame_map_points.find(mp) == current_frame_map_points.end())
      continue;
    if(!current_frame->IsInFrustum(mp))
      continue;
    //TODO: mp->IncreaseVisible();
  }*/

  //TODO:: Calculate threshold
  int threshold = 1;

  return false;
}

TrackingResult Tracker::Track(const std::shared_ptr<FrameBase> & frame) {
  switch (state_) {
    case NOT_INITIALIZED: {
      if (frame->IsValid()) {
        initial_frame_ = frame;
        last_frame_ = frame;
        last_frame_->InitializeIdentity();
        last_frame_->SetInitial(true);
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

      if (!frame->TrackWithReferenceKeyFrame(last_frame_)) {
        state_ = RECENTLY_LOST;
        return TRACKING_FAILED;
      }

      if (!frame->TrackLocalMap()) {
        return TRACK_LM_FAILED;
      }

      std::unordered_set<FrameBase *> local_frames;
      FrameBase *max_covisible_frame;
      if (nullptr == (max_covisible_frame = frame->ListLocalKeyFrames(local_frames))) {
        state_ = RECENTLY_LOST;
        break;
      }
      std::unordered_set<map::MapPoint *> local_map_points;
      FrameBase::ListAllMapPoints(local_frames, local_map_points);
      if (!TrackLocalMap(frame, local_frames, local_map_points)) {
        state_ = RECENTLY_LOST;
        break;
      }

      {

        map::Map *current_map = atlas_->GetCurrentMap();
        current_map->AddKeyFrame(frame);
        last_frame_ = frame;
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