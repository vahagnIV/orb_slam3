//
// Created by vahagn on 16/03/2021.
//

#include "local_mapper.h"
#include "frame/visible_map_point.h"
#include "frame/key_frame.h"
#include "logging.h"
#include "optimization/bundle_adjustment.h"

namespace orb_slam3 {

LocalMapper::LocalMapper(map::Atlas * atlas) : PositionObserver(), atlas_(atlas), thread_(nullptr) {}

LocalMapper::~LocalMapper() {
  Stop();
}

void LocalMapper::Run() {
  while (!cancelled_) {
    UpdateMessage message;
    GetUpdateQueue().wait_dequeue(message);
    switch (message.type) {
      case PositionMessageType::Final:continue;
      case PositionMessageType::Initial:continue;
      case PositionMessageType::Update: {
        std::cout << message.frame->Id() << std::endl;
//        CreateNewMapPoints(message.frame);
      }
    }
  }
}

void LocalMapper::Start() {
  if (thread_ != nullptr)
    return;
  cancelled_ = false;
  thread_ = new std::thread(&LocalMapper::Run, this);
}

void LocalMapper::Stop() {
  if (nullptr == thread_)
    return;
  cancelled_ = true;
  thread_->join();
  delete thread_;
  thread_ = nullptr;
}

void LocalMapper::MapPointCulling(frame::KeyFrame * keyframe) {
  unsigned erased = recently_added_map_points_.size();
  for (auto mp_it = recently_added_map_points_.begin(); mp_it != recently_added_map_points_.end(); ++mp_it) {
    map::MapPoint * mp = *mp_it;
    if (mp->IsBad()) {
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (static_cast<precision_t>(mp->GetFound()) / mp->GetVisible() < 0.25) {
      mp->SetBad();
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (keyframe->Id() > mp->GetFirstObservedFrameId() && keyframe->Id() - mp->GetFirstObservedFrameId() >= 2
        && mp->GetObservationCount() < keyframe->GetSensorConstants()->max_mp_disappearance_count) {
      mp->SetBad();
      mp_it = recently_added_map_points_.erase(mp_it);
    } else if (keyframe->Id() > mp->GetFirstObservedFrameId() && keyframe->Id() - mp->GetFirstObservedFrameId() >= 2) {
      mp_it = recently_added_map_points_.erase(mp_it);
    } else
      --erased;
  }

  logging::RetrieveLogger()->debug("LM: erased {} mps", erased);
}

void LocalMapper::EraseMapPoint(map::MapPoint * map_point) {
  for (auto obs: map_point->Observations()) {
//  TODO:  obs.first->EraseMapPoint(map_point);
//    delete obs.second;
  }
  delete map_point;
}

void LocalMapper::ProcessNewKeyFrame(frame::KeyFrame * keyframe) {
  frame::KeyFrame::MapPointSet map_points;
  keyframe->ComputeBow();
  keyframe->ListMapPoints(map_points);
  for (auto mp: map_points) {
    if (mp->IsBad())
      continue;

    if (!mp->IsInKeyFrame(keyframe)) {
      // TODO: find cases when this could happen
      assert(false);
    } else
      recently_added_map_points_.insert(mp);
  }
  keyframe->GetCovisibilityGraph().Update();
  // TODO: Add to Atlas

}

void LocalMapper::CreateNewMapPoints(frame::KeyFrame * key_frame) const {
  auto covisible_frames =
      key_frame->GetCovisibilityGraph().GetCovisibleKeyFrames(key_frame->GetSensorConstants()->number_of_keyframe_to_search_lm);

  for (auto neighbour_keyframe: covisible_frames) {
    key_frame->CreateNewMapPoints(neighbour_keyframe);
  }
}

void LocalMapper::Optimize(frame::KeyFrame * frame) {
  auto covisible_frames = frame->GetCovisibilityGraph().GetCovisibleKeyFrames();
  std::unordered_set<frame::KeyFrame *> local_keyframes;
  std::copy_if(covisible_frames.begin(),
               covisible_frames.end(),
               std::inserter(local_keyframes, local_keyframes.begin()),
               [](const frame::KeyFrame * frame) { return !frame->IsBad(); });
  local_keyframes.insert(frame);
  std:: unordered_set<frame::KeyFrame *> fixed_keyframes;
  FilterFixedKeyFames(local_keyframes, fixed_keyframes);

}

void LocalMapper::FilterFixedKeyFames(const unordered_set<frame::KeyFrame *> & keyframes,
                                      unordered_set<frame::KeyFrame *> & out_fixed) const {
  unsigned number_of_fixed = 0;
  for(auto keyframe: keyframes){
    if(keyframe->IsInitial())
      ++number_of_fixed;
  }

}

bool LocalMapper::CheckNewKeyFrames() const {
  return GetUpdateQueue().size_approx() > 0;
}

void LocalMapper::RunIteration() {
  UpdateMessage message;
  if (GetUpdateQueue().try_dequeue(message)) {
    ProcessNewKeyFrame(message.frame);
    MapPointCulling(message.frame);
    CreateNewMapPoints(message.frame);
    if (!CheckNewKeyFrames()) {
      Optimize(message.frame);
    }
  }

}

}